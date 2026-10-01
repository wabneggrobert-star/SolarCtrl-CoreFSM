#include "feature_forecast.h"
#include "feature_time.h"
#include "feature_sensor_assignments.h"
#include "feature_energy_meter.h"
#include "config.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <SD.h>
#include <math.h>

namespace {
Forecast::Summary g_summary;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool g_taskRunning = false;
volatile bool g_force = false;
uint32_t g_lastFetchMs = 0;
uint32_t g_lastCsvEpoch = 0;
uint32_t g_auxDelayStartedMs = 0;
uint32_t g_lastLearningMs = 0;
double g_lastLearningEnergyKWh = 0.0;
float g_efficiencyMean = 0.0f;
float g_next3hMean = 0.0f;
uint32_t g_learningSamples = 0;

struct Point { time_t epoch=0; float temp=NAN, cloud=NAN, precip=NAN, sunshine=NAN, gti=NAN; };
Point g_points[48];
uint8_t g_pointCount = 0;

String extractArray(const String& body, const char* key) {
  String marker = String("\"") + key + "\":[";
  int p = body.indexOf(marker); if (p < 0) return "";
  p += marker.length(); int e = body.indexOf(']', p); if (e < 0) return "";
  return body.substring(p, e);
}

uint8_t parseFloatArray(const String& a, float* out, uint8_t maxN) {
  uint8_t n=0; int start=0;
  while (start < (int)a.length() && n < maxN) {
    int comma=a.indexOf(',',start); if(comma<0) comma=a.length();
    String v=a.substring(start,comma); v.trim(); out[n++]=(v=="null"?NAN:v.toFloat()); start=comma+1;
  }
  return n;
}

uint8_t parseTimeArray(const String& a, time_t* out, uint8_t maxN) {
  uint8_t n=0; int p=0;
  while (p < (int)a.length() && n < maxN) {
    int q1=a.indexOf('"',p); if(q1<0) break; int q2=a.indexOf('"',q1+1); if(q2<0) break;
    String s=a.substring(q1+1,q2); struct tm t={};
    int yy=0,mo=0,dd=0,hh=0,mm=0;
    if (sscanf(s.c_str(), "%d-%d-%dT%d:%d", &yy,&mo,&dd,&hh,&mm) == 5) {
      t.tm_year=yy-1900; t.tm_mon=mo-1; t.tm_mday=dd; t.tm_hour=hh; t.tm_min=mm; t.tm_isdst=-1;
      out[n++]=mktime(&t);
    }
    p=q2+1;
  }
  return n;
}

float collectorTypeInitialEfficiency(const ConfigData& c) {
  return c.solarCollectorType == SolarCollectorType::EVACUATED_TUBE ? 0.55f : 0.45f;
}

float effectiveEfficiency(const ConfigData& c) {
  if (g_learningSamples >= 24 && g_efficiencyMean > 0.05f && g_efficiencyMean < 0.95f) return g_efficiencyMean;
  return collectorTypeInitialEfficiency(c);
}

void saveLearning() {
  if (!SD.exists("/runtime")) SD.mkdir("/runtime");
  File f=SD.open(FILE_FORECAST_LEARNING, FILE_WRITE); if(!f) return;
  f.printf("samples=%lu\n", (unsigned long)g_learningSamples);
  f.printf("meanEfficiency=%.6f\n", g_efficiencyMean);
  f.printf("meanNext3hKWh=%.6f\n", g_next3hMean);
  f.close();
}

void loadLearning() {
  if(!SD.exists(FILE_FORECAST_LEARNING)) return;
  File f=SD.open(FILE_FORECAST_LEARNING, FILE_READ); if(!f) return;
  while(f.available()) { String line=f.readStringUntil('\n'); line.trim(); int eq=line.indexOf('='); if(eq<0) continue; String k=line.substring(0,eq),v=line.substring(eq+1);
    if(k=="samples") g_learningSamples=(uint32_t)v.toInt(); else if(k=="meanEfficiency") g_efficiencyMean=v.toFloat(); else if(k=="meanNext3hKWh") g_next3hMean=v.toFloat();
  } f.close();
}

void updateSummaryFromPoints(const ConfigData& cfg) {
  if(g_pointCount==0) return;

  const time_t now=time(nullptr);
  float sum3=0.0f;
  int c3=0;
  float today=0.0f;
  float tomorrow=0.0f;
  float tomorrowSunshineSeconds=0.0f;
  float current=NAN;

  // Lokale Tagesgrenzen ueber mktime bilden. Das ist auch an Jahreswechseln
  // und bei Sommer-/Winterzeit robuster als tm_yday + 1.
  struct tm todayTm={};
  localtime_r(&now,&todayTm);
  todayTm.tm_hour=0;
  todayTm.tm_min=0;
  todayTm.tm_sec=0;
  todayTm.tm_isdst=-1;
  const time_t todayStart=mktime(&todayTm);

  struct tm tomorrowTm=todayTm;
  tomorrowTm.tm_mday += 1;
  tomorrowTm.tm_isdst=-1;
  const time_t tomorrowStart=mktime(&tomorrowTm);

  struct tm dayAfterTm=tomorrowTm;
  dayAfterTm.tm_mday += 1;
  dayAfterTm.tm_isdst=-1;
  const time_t dayAfterTomorrowStart=mktime(&dayAfterTm);

  for(uint8_t i=0;i<g_pointCount;i++) {
    const Point& p=g_points[i];
    if(isnan(p.gti)) continue;

    if (p.epoch <= now+3600 && p.epoch >= now-3600) current=p.gti;
    if(p.epoch>=now && p.epoch<now+3*3600) {
      sum3+=p.gti;
      c3++;
    }
    if(p.epoch>=now && p.epoch<tomorrowStart) {
      today += p.gti/1000.0f;
    }
    if(p.epoch>=tomorrowStart && p.epoch<dayAfterTomorrowStart) {
      tomorrow += p.gti/1000.0f;
      if(!isnan(p.sunshine) && p.sunshine>0.0f) tomorrowSunshineSeconds += p.sunshine;
    }
  }

  const float mean3=c3?sum3/c3:NAN;
  const float irr3=c3?sum3/1000.0f:NAN;
  const float tomorrowSunshineHours=tomorrowSunshineSeconds/3600.0f;

  portENTER_CRITICAL(&g_mux);
  g_summary.currentGtiWm2=current;
  g_summary.next3hMeanGtiWm2=mean3;
  g_summary.next3hIrradiationKWhM2=irr3;
  g_summary.todayRemainingKWhM2=today;
  g_summary.tomorrowKWhM2=tomorrow;
  g_summary.tomorrowSunshineHours=tomorrowSunshineHours;
  g_summary.expectedThermalNext3hKWh = isnan(irr3)?NAN:(irr3*max(0.1f,cfg.collectorApertureM2)*effectiveEfficiency(cfg));
  g_summary.learningSamples=g_learningSamples;
  g_summary.learnedMeanEfficiency=g_learningSamples?g_efficiencyMean:NAN;
  g_summary.learnedMeanNext3hKWh=g_learningSamples?g_next3hMean:NAN;
  g_summary.optimizerReady=false; // ML V1 entscheidet separat; Forecast bleibt Datenquelle.
  portEXIT_CRITICAL(&g_mux);
}

void ensureForecastCsv() {
  if(!SD.exists("/logs")) SD.mkdir("/logs");
  if(SD.exists(FILE_FORECAST_LOG)) return;
  File f=SD.open(FILE_FORECAST_LOG,FILE_WRITE); if(!f) return;
  f.println("logged_at,forecast_for,gti_w_m2,temp_c,cloud_pct,precip_prob_pct,sunshine_s,collector_type,aperture_m2,tilt_deg,azimuth_deg,learned_efficiency,expected_thermal_kwh_1h,collector_c,flow_l_min,power_kw,energy_kwh_total,buffer_top_c,buffer_mid_c,buffer_bottom_c,boiler_top_c,boiler_bottom_c"); f.close();
}

float roleTemp(const AppContext& ctx, Ds18Role role) {
  float t=NAN; bool valid=false;
  SensorAssignments::readByRole(ctx.assignments, role, t, valid);
  return valid ? t : NAN;
}

void csvValue(File& f, float value, uint8_t decimals=2) {
  if (!isnan(value)) f.print(value, decimals);
}

void appendForecastCsv(const AppContext& ctx) {
  const ConfigData& cfg=ctx.config;
  if(!TimeService::valid()) return; uint32_t now=(uint32_t)time(nullptr);
  if(g_lastCsvEpoch && now-g_lastCsvEpoch<21600UL) return; // 6 h snapshots, begrenzt SD-Wachstum
  ensureForecastCsv(); File f=SD.open(FILE_FORECAST_LOG,FILE_APPEND); if(!f) return;
  String logged=TimeService::isoTimestamp(); float eff=effectiveEfficiency(cfg);
  const float bufferTop=roleTemp(ctx, Ds18Role::SINK_BUFFER_TOP);
  const float bufferMid=roleTemp(ctx, Ds18Role::BUFFER_MID);
  const float bufferBottom=roleTemp(ctx, Ds18Role::BUFFER_BOTTOM);
  const float boilerTop=roleTemp(ctx, Ds18Role::SINK_BOILER_TOP);
  const float boilerBottom=roleTemp(ctx, Ds18Role::BOILER_BOTTOM);
  for(uint8_t i=0;i<g_pointCount;i++) { struct tm t={}; localtime_r(&g_points[i].epoch,&t); char ts[32]; strftime(ts,sizeof(ts),"%Y-%m-%dT%H:%M:%S%z",&t);
    f.print(logged);f.print(',');f.print(ts);f.print(',');f.print(g_points[i].gti,1);f.print(',');f.print(g_points[i].temp,1);f.print(',');f.print(g_points[i].cloud,0);f.print(',');f.print(g_points[i].precip,0);f.print(',');f.print(g_points[i].sunshine,0);f.print(',');f.print((int)cfg.solarCollectorType);f.print(',');f.print(cfg.collectorApertureM2,3);f.print(',');f.print(cfg.collectorTiltDeg,1);f.print(',');f.print(cfg.collectorAzimuthDeg,1);f.print(',');f.print(eff,4);f.print(',');f.print((g_points[i].gti/1000.0f)*cfg.collectorApertureM2*eff,5);f.print(',');csvValue(f,ctx.sensors.collectorC,2);f.print(',');f.print(ctx.energyMeter.flowLitersPerMinute,3);f.print(',');f.print(ctx.energyMeter.thermalPowerKw,3);f.print(',');f.print(ctx.energyMeter.totalEnergyKWh,5);f.print(',');csvValue(f,bufferTop);f.print(',');csvValue(f,bufferMid);f.print(',');csvValue(f,bufferBottom);f.print(',');csvValue(f,boilerTop);f.print(',');csvValue(f,boilerBottom);f.println();
  } f.close(); g_lastCsvEpoch=now;
}

void fetchTask(void* arg) {
  ConfigData cfg=*(ConfigData*)arg; delete (ConfigData*)arg;
  Forecast::Summary local=g_summary; local.fetching=true; local.lastAttemptMs=millis(); local.lastError[0]='\0';
  if(WiFi.status()!=WL_CONNECTED){strncpy(local.lastError,"kein Heim-WLAN",sizeof(local.lastError)-1);goto done;}
  {
    float apiAz=cfg.collectorAzimuthDeg-180.0f; while(apiAz>180)apiAz-=360; while(apiAz<-180)apiAz+=360;
    String url="https://api.open-meteo.com/v1/forecast?latitude="+String(cfg.forecastLatitude,5)+"&longitude="+String(cfg.forecastLongitude,5)+"&hourly=temperature_2m,cloud_cover,precipitation_probability,sunshine_duration,global_tilted_irradiance&forecast_hours=48&timezone=Europe%2FVienna&tilt="+String(cfg.collectorTiltDeg,1)+"&azimuth="+String(apiAz,1);
    WiFiClientSecure client; client.setInsecure(); HTTPClient http; http.setTimeout(8000);
    if(!http.begin(client,url)){strncpy(local.lastError,"HTTP Start fehlgeschlagen",sizeof(local.lastError)-1);goto done;}
    int code=http.GET(); if(code!=200){snprintf(local.lastError,sizeof(local.lastError),"HTTP %d",code);http.end();goto done;}
    String body=http.getString(); http.end();
    String at=extractArray(body,"time"), ag=extractArray(body,"global_tilted_irradiance"), temp=extractArray(body,"temperature_2m"), cloud=extractArray(body,"cloud_cover"), precip=extractArray(body,"precipitation_probability"), sun=extractArray(body,"sunshine_duration");
    time_t times[48]; float gti[48],tv[48],cv[48],pv[48],sv[48];
    uint8_t n=parseTimeArray(at,times,48); uint8_t ng=parseFloatArray(ag,gti,48); parseFloatArray(temp,tv,48); parseFloatArray(cloud,cv,48); parseFloatArray(precip,pv,48); parseFloatArray(sun,sv,48);
    if(n==0||ng==0){strncpy(local.lastError,"Forecast JSON unvollstaendig",sizeof(local.lastError)-1);goto done;}
    g_pointCount=min(n,ng); for(uint8_t i=0;i<g_pointCount;i++){g_points[i].epoch=times[i];g_points[i].gti=gti[i];g_points[i].temp=tv[i];g_points[i].cloud=cv[i];g_points[i].precip=pv[i];g_points[i].sunshine=sv[i];}
    local.valid=true; local.lastSuccessEpoch=TimeService::valid()?(uint32_t)time(nullptr):0; local.lastError[0]='\0';
  }
done:
  local.fetching=false; portENTER_CRITICAL(&g_mux); g_summary=local; portEXIT_CRITICAL(&g_mux); g_taskRunning=false; vTaskDelete(nullptr);
}

void updateLearning(AppContext& ctx) {
  if(!g_summary.valid || !ctx.config.energyMeter.enabled || !TimeService::valid()) return;
  uint32_t now=millis(); if(g_lastLearningMs==0){g_lastLearningMs=now;g_lastLearningEnergyKWh=ctx.energyMeter.totalEnergyKWh;return;}
  uint32_t dt=now-g_lastLearningMs; if(dt<1800000UL) return;
  double de=ctx.energyMeter.totalEnergyKWh-g_lastLearningEnergyKWh; float hours=dt/3600000.0f; float incident=(max(0.0f,g_summary.currentGtiWm2)/1000.0f)*max(0.1f,ctx.config.collectorApertureM2)*hours;
  if(incident>0.03f && de>=0.0) { float eff=constrain((float)(de/incident),0.02f,0.95f); g_learningSamples++; g_efficiencyMean += (eff-g_efficiencyMean)/g_learningSamples; if(!isnan(g_summary.expectedThermalNext3hKWh)) g_next3hMean += (g_summary.expectedThermalNext3hKWh-g_next3hMean)/g_learningSamples; saveLearning(); }
  g_lastLearningMs=now; g_lastLearningEnergyKWh=ctx.energyMeter.totalEnergyKWh; updateSummaryFromPoints(ctx.config);
}
}

namespace Forecast {
void begin(AppContext& ctx){memset(&g_summary,0,sizeof(g_summary));g_summary.enabled=ctx.config.forecastEnabled;g_summary.currentGtiWm2=NAN;g_summary.next3hMeanGtiWm2=NAN;g_summary.next3hIrradiationKWhM2=NAN;g_summary.todayRemainingKWhM2=NAN;g_summary.tomorrowKWhM2=NAN;g_summary.tomorrowSunshineHours=NAN;g_summary.expectedThermalNext3hKWh=NAN;loadLearning();g_force=true;}
void process(AppContext& ctx){g_summary.enabled=ctx.config.forecastEnabled;if(!ctx.config.forecastEnabled)return;if(g_summary.valid){updateSummaryFromPoints(ctx.config);appendForecastCsv(ctx);}uint32_t interval=max((uint32_t)900000UL,(uint32_t)ctx.config.forecastIntervalMs);if((g_force||g_lastFetchMs==0||(uint32_t)(millis()-g_lastFetchMs)>=interval)&&!g_taskRunning&&WiFi.status()==WL_CONNECTED){g_force=false;g_lastFetchMs=millis();g_taskRunning=true;ConfigData* copy=new ConfigData(ctx.config);if(xTaskCreatePinnedToCore(fetchTask,"forecast",8192,copy,1,nullptr,0)!=pdPASS){delete copy;g_taskRunning=false;}}}
void forceRefresh(){g_force=true;}
Summary summary(){portENTER_CRITICAL(&g_mux);Summary s=g_summary;portEXIT_CRITICAL(&g_mux);return s;}

bool shouldDelayAuxHeater(const AppContext&){ return false; }

bool preferBufferForSolar(const AppContext&,const PumpConfig&){ return false; }

String json(const AppContext& ctx){Summary s=summary();String j="{";j+="\"enabled\":"+String(ctx.config.forecastEnabled?"true":"false")+",";j+="\"valid\":"+String(s.valid?"true":"false")+",";j+="\"fetching\":"+String(s.fetching?"true":"false")+",";j+="\"optimizerReady\":"+String(s.optimizerReady?"true":"false")+",";j+="\"learningSamples\":"+String(s.learningSamples)+",";j+="\"lastError\":\""+String(s.lastError)+"\",";j+="\"currentGtiWm2\":"+(isnan(s.currentGtiWm2)?String("null"):String(s.currentGtiWm2,1))+",";j+="\"next3hMeanGtiWm2\":"+(isnan(s.next3hMeanGtiWm2)?String("null"):String(s.next3hMeanGtiWm2,1))+",";j+="\"next3hIrradiationKWhM2\":"+(isnan(s.next3hIrradiationKWhM2)?String("null"):String(s.next3hIrradiationKWhM2,3))+",";j+="\"todayRemainingKWhM2\":"+(isnan(s.todayRemainingKWhM2)?String("null"):String(s.todayRemainingKWhM2,3))+",";j+="\"tomorrowKWhM2\":"+(isnan(s.tomorrowKWhM2)?String("null"):String(s.tomorrowKWhM2,3))+",";j+="\"tomorrowSunshineHours\":"+(isnan(s.tomorrowSunshineHours)?String("null"):String(s.tomorrowSunshineHours,2))+",";j+="\"expectedThermalNext3hKWh\":"+(isnan(s.expectedThermalNext3hKWh)?String("null"):String(s.expectedThermalNext3hKWh,3))+",";j+="\"learnedMeanEfficiency\":"+(isnan(s.learnedMeanEfficiency)?String("null"):String(s.learnedMeanEfficiency,4))+",";j+="\"learnedMeanNext3hKWh\":"+(isnan(s.learnedMeanNext3hKWh)?String("null"):String(s.learnedMeanNext3hKWh,3))+",";j+="\"lastSuccessEpoch\":"+String(s.lastSuccessEpoch);j+="}";return j;}
}
