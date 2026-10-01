#include "feature_ml_optimizer.h"
#include "feature_forecast.h"
#include "feature_time.h"
#include "feature_sensor_assignments.h"
#include "feature_safety_manager.h"
#include "feature_fluid_properties.h"
#include "feature_energy_meter.h"
#include "config.h"
#include <SD.h>
#include <math.h>
#include <string.h>
#include <time.h>

namespace {
constexpr uint8_t FEATURE_COUNT = 6;
constexpr uint8_t MAX_PENDING = 12;
constexpr uint16_t MAX_ERROR_WINDOW = 256;
constexpr uint32_t SAMPLE_INTERVAL_MS = 1800000UL; // 30 min
constexpr uint32_t PREDICTION_HORIZON_S = 3UL*3600UL;
constexpr uint32_t WINDOW_7D_S = 7UL*86400UL;
constexpr uint32_t MIN_TRAINING_SAMPLES = 200;
constexpr uint16_t MIN_TRAINING_DAYS = 7;
constexpr uint16_t MIN_WINDOW_SAMPLES = 50;
constexpr float MAX_WINDOW_ERROR_PCT = 10.0f;
constexpr float CONTROL_CONFIDENCE_FACTOR = 0.85f; // conservative by design
constexpr float SGD_RATE = 0.035f;

struct PendingSample {
  bool used=false;
  bool valid=true;
  uint32_t createdEpoch=0;
  uint32_t dueEpoch=0;
  double startEnergyKWh=0.0;
  float predictedKWh=NAN;
  float features[FEATURE_COUNT] = {};
  char invalidReason[28]="";
};
struct ErrorPoint { uint32_t epoch=0; float errorPct=NAN; };

MlOptimizer::Status g_status;
PendingSample g_pending[MAX_PENDING];
ErrorPoint g_errors[MAX_ERROR_WINDOW];
uint16_t g_errorHead=0, g_errorCount=0;
float g_weights[FEATURE_COUNT] = {};
float g_hcBias[MAX_HEATING_CIRCUITS] = {};
float g_hcOutsideCoeff[MAX_HEATING_CIRCUITS] = {};
uint32_t g_trainingSamples=0;
uint16_t g_trainingDays=0;
int32_t g_lastTrainingDayKey=-1;
uint32_t g_lastCreateMs=0;
uint32_t g_lastRoomLearnMs=0;
uint32_t g_auxDelayStartedMs=0;
bool g_loaded=false;

float clampf(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}

bool readRole(const AppContext& ctx, Ds18Role role, float& value) {
  value=NAN; bool valid=false;
  if(role==Ds18Role::NONE) return false;
  return SensorAssignments::readByRole(ctx.assignments,role,value,valid) && valid && !isnan(value);
}

bool isSolarRole(HeatSourceRole r){return r==HeatSourceRole::SOLAR_COLLECTOR_1||r==HeatSourceRole::SOLAR_COLLECTOR_2||r==HeatSourceRole::SOLAR_COLLECTOR_3;}
bool isBoilerRole(Ds18Role r){return r==Ds18Role::SINK_BOILER_TOP||r==Ds18Role::BOILER_BOTTOM;}
bool isBufferRole(Ds18Role r){return r==Ds18Role::SINK_BUFFER_TOP||r==Ds18Role::BUFFER_HIGH||r==Ds18Role::BUFFER_MID||r==Ds18Role::BUFFER_BOTTOM;}

int findSolarPump(const AppContext& ctx) {
  for(uint8_t i=0;i<MAX_PUMPS;i++){
    const PumpConfig& p=ctx.config.pumps[i];
    if(p.enabled && p.sourceType==PumpSourceType::HEAT_SOURCE_ROLE && isSolarRole(p.sourceRole)) return i;
  }
  return -1;
}

int32_t localDayKey(uint32_t epoch){
  time_t t=(time_t)epoch; struct tm tmv={}; localtime_r(&t,&tmv);
  return (tmv.tm_year+1900)*1000 + tmv.tm_yday;
}

float daySin(uint32_t epoch){time_t t=epoch;struct tm tmv={};localtime_r(&t,&tmv);return sinf(2.0f*PI*(float)tmv.tm_yday/365.25f);}
float dayCos(uint32_t epoch){time_t t=epoch;struct tm tmv={};localtime_r(&t,&tmv);return cosf(2.0f*PI*(float)tmv.tm_yday/365.25f);}

float roleMeanStorage(const AppContext& ctx){
  float a=NAN,b=NAN; bool va=readRole(ctx,Ds18Role::SINK_BUFFER_TOP,a); bool vb=readRole(ctx,Ds18Role::SINK_BOILER_TOP,b);
  if(va&&vb) return 0.5f*(a+b); if(va)return a; if(vb)return b; return 45.0f;
}
float outsideTemp(const AppContext& ctx){float v=NAN;return readRole(ctx,Ds18Role::OUTSIDE_TEMPERATURE,v)?v:10.0f;}

void featuresFor(const AppContext& ctx, float out[FEATURE_COUNT], float& incidentKWh) {
  const Forecast::Summary fs=Forecast::summary();
  const float irr=isnan(fs.next3hIrradiationKWhM2)?0.0f:max(0.0f,fs.next3hIrradiationKWhM2);
  incidentKWh=irr*max(0.1f,ctx.config.collectorApertureM2);
  const float outC=outsideTemp(ctx);
  const float storage=roleMeanStorage(ctx);
  const int spi=findSolarPump(ctx);
  const float pwm=(spi>=0)?ctx.config.pumps[spi].lastPwmPercent:50.0f;
  const uint32_t now=TimeService::valid()?(uint32_t)time(nullptr):0;
  out[0]=incidentKWh;
  out[1]=incidentKWh*clampf((outC-10.0f)/30.0f,-1.0f,1.5f);
  out[2]=incidentKWh*clampf((storage-45.0f)/45.0f,-1.0f,1.2f);
  out[3]=incidentKWh*(now?daySin(now):0.0f);
  out[4]=incidentKWh*(now?dayCos(now):0.0f);
  out[5]=incidentKWh*clampf((pwm-50.0f)/50.0f,-1.0f,1.0f);
}

float rawPredict(const float x[FEATURE_COUNT], float incidentKWh){
  float y=0.0f; for(uint8_t i=0;i<FEATURE_COUNT;i++) y+=g_weights[i]*x[i];
  return clampf(y,0.0f,max(0.0f,incidentKWh*0.95f));
}

void resetWeights(const ConfigData& cfg){
  for(float &w:g_weights) w=0.0f;
  g_weights[0]=(cfg.solarCollectorType==SolarCollectorType::EVACUATED_TUBE)?0.55f:0.45f;
  for(uint8_t i=0;i<MAX_HEATING_CIRCUITS;i++){g_hcBias[i]=0.0f;g_hcOutsideCoeff[i]=0.0f;}
}

void saveRuntime(){
  if(!SD.exists("/runtime")) SD.mkdir("/runtime");
  File f=SD.open(FILE_ML_RUNTIME,FILE_WRITE); if(!f)return;
  f.printf("samples=%lu\n",(unsigned long)g_trainingSamples);
  f.printf("days=%u\n",g_trainingDays); f.printf("lastDay=%ld\n",(long)g_lastTrainingDayKey);
  for(uint8_t i=0;i<FEATURE_COUNT;i++)f.printf("w%u=%.8f\n",i,g_weights[i]);
  for(uint8_t i=0;i<MAX_HEATING_CIRCUITS;i++){f.printf("hcBias%u=%.5f\n",i,g_hcBias[i]);f.printf("hcOutside%u=%.6f\n",i,g_hcOutsideCoeff[i]);}
  f.printf("errorCount=%u\n",g_errorCount);
  for(uint16_t j=0;j<g_errorCount;j++){uint16_t idx=(g_errorHead+MAX_ERROR_WINDOW-g_errorCount+j)%MAX_ERROR_WINDOW;f.printf("e%u=%lu,%.4f\n",j,(unsigned long)g_errors[idx].epoch,g_errors[idx].errorPct);}
  f.close();
}

void loadRuntime(const ConfigData& cfg){
  resetWeights(cfg); g_trainingSamples=0;g_trainingDays=0;g_lastTrainingDayKey=-1;g_errorHead=0;g_errorCount=0;
  if(!SD.exists(FILE_ML_RUNTIME)){g_loaded=true;return;}
  File f=SD.open(FILE_ML_RUNTIME,FILE_READ); if(!f){g_loaded=true;return;}
  while(f.available()){
    String line=f.readStringUntil('\n');line.trim();int eq=line.indexOf('=');if(eq<0)continue;String k=line.substring(0,eq),v=line.substring(eq+1);
    if(k=="samples")g_trainingSamples=(uint32_t)v.toInt(); else if(k=="days")g_trainingDays=(uint16_t)v.toInt(); else if(k=="lastDay")g_lastTrainingDayKey=v.toInt();
    else if(k.startsWith("w")){int i=k.substring(1).toInt();if(i>=0&&i<FEATURE_COUNT)g_weights[i]=v.toFloat();}
    else if(k.startsWith("hcBias")){int i=k.substring(6).toInt();if(i>=0&&i<MAX_HEATING_CIRCUITS)g_hcBias[i]=v.toFloat();}
    else if(k.startsWith("hcOutside")){int i=k.substring(9).toInt();if(i>=0&&i<MAX_HEATING_CIRCUITS)g_hcOutsideCoeff[i]=v.toFloat();}
    else if(k.startsWith("e")){
      int comma=v.indexOf(',');if(comma>0&&g_errorCount<MAX_ERROR_WINDOW){ErrorPoint ep;ep.epoch=(uint32_t)v.substring(0,comma).toInt();ep.errorPct=v.substring(comma+1).toFloat();g_errors[g_errorHead]=ep;g_errorHead=(g_errorHead+1)%MAX_ERROR_WINDOW;g_errorCount++;}
    }
  }f.close();g_loaded=true;
}

void appendTrainingLog(uint32_t created,uint32_t due,float predicted,float measured,float err,const char* result){
  if(!SD.exists("/logs"))SD.mkdir("/logs"); bool header=!SD.exists(FILE_ML_LOG); File f=SD.open(FILE_ML_LOG,FILE_APPEND);if(!f)return;
  if(header||f.size()==0)f.println("completed_at,prediction_created_at,prediction_until,predicted_kwh,measured_kwh,error_pct,result,model_version,training_samples");
  time_t now=time(nullptr),ct=created,dt=due;struct tm tmv={};char a[32],b[32],c[32];localtime_r(&now,&tmv);strftime(a,sizeof(a),"%Y-%m-%dT%H:%M:%S%z",&tmv);localtime_r(&ct,&tmv);strftime(b,sizeof(b),"%Y-%m-%dT%H:%M:%S%z",&tmv);localtime_r(&dt,&tmv);strftime(c,sizeof(c),"%Y-%m-%dT%H:%M:%S%z",&tmv);
  f.print(a);f.print(',');f.print(b);f.print(',');f.print(c);f.print(',');f.print(predicted,4);f.print(',');f.print(measured,4);f.print(',');if(!isnan(err))f.print(err,2);f.print(',');f.print(result);f.print(",mlv1,");f.println(g_trainingSamples);f.close();
}

void addError(uint32_t epoch,float e){g_errors[g_errorHead]={epoch,e};g_errorHead=(g_errorHead+1)%MAX_ERROR_WINDOW;if(g_errorCount<MAX_ERROR_WINDOW)g_errorCount++;}

void calculateWindow(uint32_t now,uint16_t& count,float& mean){count=0;mean=NAN;float sum=0;for(uint16_t j=0;j<g_errorCount;j++){uint16_t idx=(g_errorHead+MAX_ERROR_WINDOW-g_errorCount+j)%MAX_ERROR_WINDOW;const auto& ep=g_errors[idx];if(ep.epoch&&now>=ep.epoch&&now-ep.epoch<=WINDOW_7D_S&&!isnan(ep.errorPct)){sum+=ep.errorPct;count++;}}if(count)mean=sum/count;}

bool storageLimited(const AppContext& ctx,int pumpIndex){
  if(pumpIndex<0)return true;const PumpConfig& p=ctx.config.pumps[pumpIndex];
  bool anyTarget=false; bool anyAvailable=false;
  for(uint8_t i=0;i<PUMP_ROUTE_TARGET_COUNT;i++){
    const auto&t=p.targets[i];if(!t.enabled||t.sinkRole==Ds18Role::NONE)continue;float v=NAN;if(!readRole(ctx,t.sinkRole,v))continue;
    anyTarget=true; if(t.maxTempC<=0.01f || v<t.maxTempC-0.3f) anyAvailable=true;
  }
  return anyTarget && !anyAvailable;
}

bool commonTrainingValid(const AppContext& ctx,bool testActive,char* reason,size_t reasonSize){
  if(testActive){strncpy(reason,"test_mode",reasonSize);return false;}
  if(!ctx.config.forecastEnabled||!Forecast::summary().valid){strncpy(reason,"forecast_invalid",reasonSize);return false;}
  if(!TimeService::valid()){strncpy(reason,"time_invalid",reasonSize);return false;}
  if(!ctx.config.energyMeter.enabled){strncpy(reason,"energy_meter_off",reasonSize);return false;}
  const auto& st=SafetyManager::status();if(st.blockNormalPumpControl||st.storageCriticalOvertemperatureActive||st.sensorFaultActive){strncpy(reason,"safety_active",reasonSize);return false;}
  const int sp=findSolarPump(ctx);if(sp<0){strncpy(reason,"no_solar_pump",reasonSize);return false;}
  if(storageLimited(ctx,sp)){strncpy(reason,"storage_limited",reasonSize);return false;}
  if(ctx.config.collectorApertureM2<0.1f){strncpy(reason,"aperture_invalid",reasonSize);return false;}
  reason[0]='\0';return true;
}

void invalidatePending(const char* reason){for(auto &p:g_pending)if(p.used&&p.valid){p.valid=false;strncpy(p.invalidReason,reason,sizeof(p.invalidReason)-1);}}

void maturePending(const AppContext& ctx,uint32_t now){
  for(auto &p:g_pending){
    if(!p.used||now<p.dueEpoch)continue;
    const float measured=(float)(ctx.energyMeter.totalEnergyKWh-p.startEnergyKWh);
    if(!p.valid||measured<0.0f){appendTrainingLog(p.createdEpoch,p.dueEpoch,p.predictedKWh,max(0.0f,measured),NAN,p.invalidReason[0]?p.invalidReason:"invalid");p.used=false;continue;}
    const float denom=max(0.25f,measured);const float err=fabsf(p.predictedKWh-measured)/denom*100.0f;
    // SGD update. Normalize by incident-like feature magnitude to avoid unstable steps.
    float norm=0.1f;for(float x:p.features)norm+=x*x;const float delta=measured-p.predictedKWh;
    for(uint8_t i=0;i<FEATURE_COUNT;i++){g_weights[i]+=SGD_RATE*delta*p.features[i]/norm;g_weights[i]=clampf(g_weights[i],-0.8f,1.2f);}
    g_weights[0]=clampf(g_weights[0],0.05f,0.90f);
    g_trainingSamples++;const int32_t dk=localDayKey(now);if(dk!=g_lastTrainingDayKey){g_trainingDays++;g_lastTrainingDayKey=dk;}
    addError(now,err);g_status.lastMeasuredKWh=measured;g_status.lastPredictionErrorPercent=err;
    appendTrainingLog(p.createdEpoch,p.dueEpoch,p.predictedKWh,measured,err,"valid");p.used=false;saveRuntime();
  }
}

void maybeCreate(const AppContext& ctx,bool testActive,uint32_t now){
  if(g_lastCreateMs&&(uint32_t)(millis()-g_lastCreateMs)<SAMPLE_INTERVAL_MS)return;
  char reason[28];if(!commonTrainingValid(ctx,testActive,reason,sizeof(reason)))return;
  float x[FEATURE_COUNT],incident=0;featuresFor(ctx,x,incident);if(incident<0.05f)return;
  int slot=-1;for(uint8_t i=0;i<MAX_PENDING;i++)if(!g_pending[i].used){slot=i;break;}if(slot<0)return;
  PendingSample &p=g_pending[slot];p=PendingSample{};p.used=true;p.valid=true;p.createdEpoch=now;p.dueEpoch=now+PREDICTION_HORIZON_S;p.startEnergyKWh=ctx.energyMeter.totalEnergyKWh;for(uint8_t i=0;i<FEATURE_COUNT;i++)p.features[i]=x[i];p.predictedKWh=rawPredict(x,incident);g_lastCreateMs=millis();
}

void learnHeatingCircuits(const AppContext& ctx,bool testActive){
  if(testActive || SafetyManager::status().blockNormalPumpControl)return;const uint32_t nowMs=millis();if(g_lastRoomLearnMs&&(uint32_t)(nowMs-g_lastRoomLearnMs)<SAMPLE_INTERVAL_MS)return;g_lastRoomLearnMs=nowMs;
  for(uint8_t i=0;i<MAX_HEATING_CIRCUITS;i++){
    const auto& c=ctx.config.heatingCircuits[i];const auto& rt=ctx.heatingCircuitRuntime[i];if(!c.enabled||!c.roomControlEnabled||isnan(rt.roomTemperatureC))continue;
    const float err=c.roomTargetTemperatureC-rt.roomTemperatureC;const float outside=isnan(rt.outsideTemperatureC)?10.0f:rt.outsideTemperatureC;const float xn=clampf((20.0f-outside)/30.0f,-0.5f,1.5f);
    g_hcBias[i]=clampf(g_hcBias[i]+0.025f*err,-4.0f,4.0f);g_hcOutsideCoeff[i]=clampf(g_hcOutsideCoeff[i]+0.0025f*err*xn,-1.5f,1.5f);
  }
}

float storageVolumeForRole(const ConfigData& c,Ds18Role role){if(isBoilerRole(role))return c.boilerVolumeLiters;if(isBufferRole(role))return c.bufferVolumeLiters;return 0.0f;}
float requiredStorageEnergyKWh(const ConfigData& c,Ds18Role role,float current,float target){const float v=storageVolumeForRole(c,role);if(v<=1.0f||target<=current)return 0.0f;return v*(target-current)*1.163f/1000.0f;}

bool readiness(const AppContext& ctx,uint16_t &wc,float& mean){const uint32_t now=TimeService::valid()?(uint32_t)time(nullptr):0;calculateWindow(now,wc,mean);const Forecast::Summary fs=Forecast::summary();return ctx.config.mlMode==MlMode::AUTOMATIC&&ctx.config.forecastEnabled&&fs.valid&&TimeService::valid()&&ctx.config.energyMeter.enabled&&g_trainingSamples>=MIN_TRAINING_SAMPLES&&g_trainingDays>=MIN_TRAINING_DAYS&&wc>=MIN_WINDOW_SAMPLES&&!isnan(mean)&&mean<MAX_WINDOW_ERROR_PCT;}

void refreshStatus(const AppContext& ctx){
  float x[FEATURE_COUNT],incident=0;featuresFor(ctx,x,incident);const float pred=rawPredict(x,incident);uint16_t wc=0;float mean=NAN;const bool ready=readiness(ctx,wc,mean);
  g_status.enabled=ctx.config.mlMode!=MlMode::OFF;g_status.learning=ctx.config.mlMode!=MlMode::OFF;g_status.active=ready;g_status.fallback=ctx.config.mlMode==MlMode::AUTOMATIC&&!ready&&g_trainingSamples>=MIN_TRAINING_SAMPLES;
  g_status.trainingSamples=g_trainingSamples;g_status.trainingDays=g_trainingDays;g_status.windowSamples7d=wc;g_status.meanError7dPercent=mean;g_status.predictedNext3hKWh=pred;g_status.conservativeNext3hKWh=pred*CONTROL_CONFIDENCE_FACTOR;
  if(ctx.config.mlMode==MlMode::OFF)snprintf(g_status.recommendation,sizeof(g_status.recommendation),"ML ist ausgeschaltet. Grundregelung bleibt aktiv.");
  else if(!ready)snprintf(g_status.recommendation,sizeof(g_status.recommendation),"Noch keine Wartezeit-Empfehlung: Lernphase %lu/200 Punkte, %u/7 Tage.",(unsigned long)g_trainingSamples,g_trainingDays);
  else snprintf(g_status.recommendation,sizeof(g_status.recommendation),"ML stabil (7-Tage-Fehler %.1f%%). Eingestellte Zusatzheizungs-Wartezeit %.1f h beibehalten und nur nach eigener Anlagenerfahrung aendern.",mean,ctx.config.forecastAuxMaxWaitMs/3600000.0f);
}
}

namespace MlOptimizer {
void begin(AppContext& ctx){memset(&g_status,0,sizeof(g_status));for(auto&p:g_pending)p=PendingSample{};loadRuntime(ctx.config);refreshStatus(ctx);}

void process(AppContext& ctx,bool testActive){
  if(!g_loaded)loadRuntime(ctx.config);
  if(ctx.config.mlMode==MlMode::OFF){refreshStatus(ctx);return;}
  const uint32_t now=TimeService::valid()?(uint32_t)time(nullptr):0;
  if(!now){refreshStatus(ctx);return;}
  char reason[28];if(!commonTrainingValid(ctx,testActive,reason,sizeof(reason)))invalidatePending(reason);
  // If the solar pump is running, invalid meter temperatures must never teach the model zero energy.
  const int sp=findSolarPump(ctx);if(sp>=0&&ctx.config.pumps[sp].state&&(!ctx.energyMeter.temperatureValid||ctx.energyMeter.flowLitersPerMinute<0.01f))invalidatePending("meter_invalid");
  maturePending(ctx,now);maybeCreate(ctx,testActive,now);learnHeatingCircuits(ctx,testActive);refreshStatus(ctx);
}

void resetLearning(AppContext& ctx){
  for(auto&p:g_pending)p=PendingSample{};g_errorHead=0;g_errorCount=0;g_trainingSamples=0;g_trainingDays=0;g_lastTrainingDayKey=-1;g_lastCreateMs=0;g_lastRoomLearnMs=0;g_auxDelayStartedMs=0;g_status.lastMeasuredKWh=NAN;g_status.lastPredictionErrorPercent=NAN;resetWeights(ctx.config);if(SD.exists(FILE_ML_RUNTIME))SD.remove(FILE_ML_RUNTIME);if(SD.exists(FILE_FORECAST_LEARNING))SD.remove(FILE_FORECAST_LEARNING);saveRuntime();refreshStatus(ctx);
}
Status status(){return g_status;}

bool shouldDelayAuxHeater(const AppContext& ctx,float currentC){
  const Status s=g_status;
  if(!s.active||!ctx.config.forecastAuxDelayEnabled||isnan(s.conservativeNext3hKWh)){g_auxDelayStartedMs=0;return false;}
  const auto& a=ctx.config.auxHeater;const float need=requiredStorageEnergyKWh(ctx.config,a.sinkRole,currentC,a.minimumTemperatureC);
  if(need<=0.01f){g_auxDelayStartedMs=0;return false;}
  const bool opportunity=s.conservativeNext3hKWh>=need;
  if(!opportunity){g_auxDelayStartedMs=0;snprintf(g_status.lastDecision,sizeof(g_status.lastDecision),"Zusatzheizung Start: freigeben (Solar %.2f kWh, Bedarf %.2f kWh)",s.conservativeNext3hKWh,need);return false;}
  const uint32_t nowMs=millis();if(g_auxDelayStartedMs==0)g_auxDelayStartedMs=nowMs;
  const uint32_t maxWait=max((uint32_t)60000UL,(uint32_t)ctx.config.forecastAuxMaxWaitMs);
  if((uint32_t)(nowMs-g_auxDelayStartedMs)>=maxWait){g_auxDelayStartedMs=0;snprintf(g_status.lastDecision,sizeof(g_status.lastDecision),"Zusatzheizung Start: maximale Wartezeit erreicht");return false;}
  snprintf(g_status.lastDecision,sizeof(g_status.lastDecision),"Zusatzheizung Start: warten (Solar %.2f kWh, Bedarf %.2f kWh)",s.conservativeNext3hKWh,need);return true;
}

bool shouldStopAuxHeaterEarly(const AppContext& ctx,float currentC){
  const Status s=g_status;if(!s.active||!ctx.config.forecastAuxDelayEnabled||isnan(s.conservativeNext3hKWh))return false;
  const auto& a=ctx.config.auxHeater;if(currentC<a.minimumTemperatureC)return false;
  const float target=a.targetTemperatureC+max(0.0f,a.hysteresisC);const float need=requiredStorageEnergyKWh(ctx.config,a.sinkRole,currentC,target);
  if(need<=0.01f)return false;const bool result=s.conservativeNext3hKWh>=need*1.10f;
  if(result)snprintf(g_status.lastDecision,sizeof(g_status.lastDecision),"Zusatzheizung frueh AUS: Solar %.2f kWh >= Bedarf %.2f kWh",s.conservativeNext3hKWh,need);
  return result;
}

bool preferBufferForSolar(const AppContext& ctx,const PumpConfig& pump){
  if(!g_status.active||!ctx.config.forecastSolarPriorityEnabled||pump.sourceType!=PumpSourceType::HEAT_SOURCE_ROLE||!isSolarRole(pump.sourceRole))return false;
  int boiler=-1,buffer=-1;for(uint8_t i=0;i<PUMP_ROUTE_TARGET_COUNT;i++){if(!pump.targets[i].enabled)continue;if(isBoilerRole(pump.targets[i].sinkRole))boiler=i;if(isBufferRole(pump.targets[i].sinkRole))buffer=i;}
  if(boiler<0||buffer<0)return false;
  float boilerC=NAN;if(!readRole(ctx,pump.targets[boiler].sinkRole,boilerC))return false;
  const float boilerMin=pump.targets[boiler].minTempC;
  // If no minimum was configured, do not let ML override the user's normal priority.
  if(boilerMin<=0.01f||boilerC<boilerMin)return false;
  float bufferC=NAN;if(!readRole(ctx,pump.targets[buffer].sinkRole,bufferC))return false;
  if(pump.targets[buffer].maxTempC>0.01f&&bufferC>=pump.targets[buffer].maxTempC)return false;
  const bool result=!isnan(g_status.conservativeNext3hKWh)&&g_status.conservativeNext3hKWh>0.25f;
  if(result)snprintf(g_status.lastDecision,sizeof(g_status.lastDecision),"Solarziel: Puffer (Boiler %.1f >= Minimum %.1f C)",boilerC,boilerMin);
  return result;
}

float adjustSolarPumpPwm(const AppContext& ctx,uint8_t pumpIndex,float requested){
  if(!g_status.active||pumpIndex>=MAX_PUMPS)return requested;const auto&p=ctx.config.pumps[pumpIndex];if(!isSolarRole(p.sourceRole)||p.mode!=PumpMode::PWM)return requested;
  // Learned PWM coefficient only contributes a small bounded correction. PID and user min/max remain authoritative.
  const float correction=clampf(g_weights[5]*12.0f,-10.0f,10.0f);return clampf(requested+correction,p.minPwmPercent,p.maxPwmPercent);
}

float adjustHeatingTarget(const AppContext& ctx,uint8_t i,float requested,float roomC,float outsideC){
  if(!g_status.active||i>=MAX_HEATING_CIRCUITS||isnan(roomC))return requested;const auto&c=ctx.config.heatingCircuits[i];
  const float xn=isnan(outsideC)?0.0f:clampf((20.0f-outsideC)/30.0f,-0.5f,1.5f);const float offset=clampf(g_hcBias[i]+g_hcOutsideCoeff[i]*xn,-5.0f,5.0f);
  return clampf(requested+offset,c.minimumFlowTemperatureC,c.maximumFlowTemperatureC);
}

String json(const AppContext& ctx){
  const Status s=g_status;String j="{";j+="\"mode\":"+String((int)ctx.config.mlMode)+",";j+="\"active\":"+String(s.active?"true":"false")+",";j+="\"learning\":"+String(s.learning?"true":"false")+",";j+="\"fallback\":"+String(s.fallback?"true":"false")+",";j+="\"trainingSamples\":"+String(s.trainingSamples)+",";j+="\"trainingDays\":"+String(s.trainingDays)+",";j+="\"windowSamples7d\":"+String(s.windowSamples7d)+",";
  j+="\"meanError7dPercent\":"+(isnan(s.meanError7dPercent)?String("null"):String(s.meanError7dPercent,2))+",";j+="\"predictedNext3hKWh\":"+(isnan(s.predictedNext3hKWh)?String("null"):String(s.predictedNext3hKWh,3))+",";j+="\"conservativeNext3hKWh\":"+(isnan(s.conservativeNext3hKWh)?String("null"):String(s.conservativeNext3hKWh,3))+",";j+="\"lastMeasuredKWh\":"+(isnan(s.lastMeasuredKWh)?String("null"):String(s.lastMeasuredKWh,3))+",";j+="\"lastPredictionErrorPercent\":"+(isnan(s.lastPredictionErrorPercent)?String("null"):String(s.lastPredictionErrorPercent,2))+",";j+="\"lastDecision\":\""+String(s.lastDecision)+"\",";j+="\"recommendation\":\""+String(s.recommendation)+"\"}";return j;
}
}
