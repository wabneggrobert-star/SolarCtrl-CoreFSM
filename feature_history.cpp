#include "feature_history.h"
#include "feature_time.h"
#include "feature_sensor_roles.h"
#include "feature_heat_source_roles.h"
#include "feature_sensor_assignments.h"
#include "feature_heat_source_assignments.h"
#include "feature_sink_ds18b20.h"
#include <SD.h>
#include <WebServer.h>
#include <time.h>
#include <math.h>

namespace {
constexpr const char* CFG_PATH = "/config/history.cfg";
constexpr uint32_t DEFAULT_INTERVAL_MS = 60000UL;
constexpr uint8_t MAX_TRACKED = 24; // 4 MAX + 20 DS18B20
constexpr uint8_t MAX_PREFS = 32;

struct Pref { char id[28] = ""; bool show = false; };
struct Agg {
  bool used = false;
  char id[28] = "";
  char label[56] = "";
  double sum = 0.0;
  float minV = NAN;
  float maxV = NAN;
  uint32_t count = 0;
};
struct Bucket {
  time_t start = 0;
  uint32_t seconds = 0;
  Agg sensors[MAX_TRACKED];
  double energyWh = 0.0;
};

bool g_enabled = false;
uint32_t g_intervalMs = DEFAULT_INTERVAL_MS;
uint32_t g_lastSampleMs = 0;
Pref g_prefs[MAX_PREFS];
uint8_t g_prefCount = 0;
Bucket g_5m, g_30m, g_day;
double g_lastEnergyKWh = NAN;

String jsonEscape(const String& in) {
  String out; out.reserve(in.length()+8);
  for (size_t i=0;i<in.length();++i) {
    char c=in[i];
    if (c=='\\' || c=='\"') { out+='\\'; out+=c; }
    else if (c=='\n') out += "\\n";
    else if (c=='\r') out += "\\r";
    else out += c;
  }
  return out;
}

void ensureDirs() {
  if (!SD.exists("/logs")) SD.mkdir("/logs");
  if (!SD.exists("/logs/history")) SD.mkdir("/logs/history");
  if (!SD.exists("/logs/history/day")) SD.mkdir("/logs/history/day");
  if (!SD.exists("/logs/history/month")) SD.mkdir("/logs/history/month");
  if (!SD.exists("/logs/history/year")) SD.mkdir("/logs/history/year");
  if (!SD.exists("/config")) SD.mkdir("/config");
}

Pref* findPref(const char* id, bool create) {
  for (uint8_t i=0;i<g_prefCount;i++) if (strcmp(g_prefs[i].id,id)==0) return &g_prefs[i];
  if (!create || g_prefCount>=MAX_PREFS) return nullptr;
  Pref& p=g_prefs[g_prefCount++];
  strncpy(p.id,id,sizeof(p.id)-1); p.id[sizeof(p.id)-1]='\0'; p.show=false;
  return &p;
}

void loadPrefs() {
  g_prefCount=0; g_enabled=false; g_intervalMs=DEFAULT_INTERVAL_MS;
  if (!SD.exists(CFG_PATH)) return;
  File f=SD.open(CFG_PATH,FILE_READ); if(!f)return;
  while(f.available()) {
    String line=f.readStringUntil('\n'); line.trim(); if(!line.length()||line[0]=='#')continue;
    int eq=line.indexOf('='); if(eq<1)continue;
    String k=line.substring(0,eq), v=line.substring(eq+1); k.trim(); v.trim();
    if(k=="enabled") g_enabled=v.toInt()!=0;
    else if(k=="intervalMs") { uint32_t x=(uint32_t)v.toInt(); if(x>=10000UL && x<=3600000UL)g_intervalMs=x; }
    else if(k.startsWith("show_")) { String id=k.substring(5); Pref* p=findPref(id.c_str(),true); if(p)p->show=v.toInt()!=0; }
  }
  f.close();
}

bool savePrefsInternal() {
  ensureDirs(); SD.remove(CFG_PATH); File f=SD.open(CFG_PATH,FILE_WRITE); if(!f)return false;
  f.printf("enabled=%u\n",g_enabled?1:0); f.printf("intervalMs=%lu\n",(unsigned long)g_intervalMs);
  for(uint8_t i=0;i<g_prefCount;i++) f.printf("show_%s=%u\n",g_prefs[i].id,g_prefs[i].show?1:0);
  f.close(); return true;
}

void localParts(time_t t, struct tm& lt) { localtime_r(&t,&lt); }
time_t floorLocal(time_t now, uint32_t sec) {
  struct tm lt; localParts(now,lt);
  if(sec==86400UL){ lt.tm_hour=0;lt.tm_min=0;lt.tm_sec=0; return mktime(&lt); }
  uint32_t sod=lt.tm_hour*3600UL+lt.tm_min*60UL+lt.tm_sec;
  uint32_t flo=(sod/sec)*sec;
  lt.tm_hour=flo/3600UL; lt.tm_min=(flo%3600UL)/60UL; lt.tm_sec=flo%60UL;
  return mktime(&lt);
}

String tsLocal(time_t t) { struct tm lt; localParts(t,lt); char b[24]; strftime(b,sizeof(b),"%Y-%m-%dT%H:%M:%S",&lt); return String(b); }
String dayPath(time_t t){struct tm lt;localParts(t,lt);char b[48];strftime(b,sizeof(b),"/logs/history/day/%Y-%m-%d.csv",&lt);return String(b);} 
String monthPath(time_t t){struct tm lt;localParts(t,lt);char b[48];strftime(b,sizeof(b),"/logs/history/month/%Y-%m.csv",&lt);return String(b);} 
String yearPath(time_t t){struct tm lt;localParts(t,lt);char b[48];strftime(b,sizeof(b),"/logs/history/year/%Y.csv",&lt);return String(b);} 

void resetAgg(Agg& a){a.used=false;a.id[0]=0;a.label[0]=0;a.sum=0;a.minV=NAN;a.maxV=NAN;a.count=0;}
void resetBucket(Bucket& b,time_t start,uint32_t sec){b.start=start;b.seconds=sec;b.energyWh=0;for(auto &a:b.sensors)resetAgg(a);} 

Agg* aggFor(Bucket& b,const char* id,const char* label){
  for(auto &a:b.sensors) if(a.used && strcmp(a.id,id)==0) return &a;
  for(auto &a:b.sensors) if(!a.used){a.used=true;strncpy(a.id,id,sizeof(a.id)-1);strncpy(a.label,label,sizeof(a.label)-1);return &a;}
  return nullptr;
}
void addValue(Bucket& b,const char* id,const char* label,float v){ if(!isfinite(v))return; Agg* a=aggFor(b,id,label);if(!a)return;a->sum+=v;a->count++;if(!isfinite(a->minV)||v<a->minV)a->minV=v;if(!isfinite(a->maxV)||v>a->maxV)a->maxV=v; }

void ensureHeader(File& f, bool fresh){ if(fresh) f.println("timestamp,type,id,label,avg,min,max,energy_wh"); }
void flushBucket(Bucket& b, const String& path){
  if(!b.start)return; ensureDirs(); bool fresh=!SD.exists(path.c_str()); File f=SD.open(path.c_str(),FILE_APPEND); if(!f)return; ensureHeader(f,fresh); String ts=tsLocal(b.start);
  for(auto &a:b.sensors){ if(!a.used||!a.count)continue; f.print(ts);f.print(",temp,");f.print(a.id);f.print(',');f.print(a.label);f.print(',');f.print(a.sum/a.count,3);f.print(',');f.print(a.minV,3);f.print(',');f.print(a.maxV,3);f.println(','); }
  if(fabs(b.energyWh)>0.000001){ f.print(ts);f.print(",energy,solar_energy,Solarenergie,,,,");f.println(b.energyWh,3); }
  f.close();
}

void advanceBucket(Bucket& b,time_t newStart,uint32_t sec,const String& oldPath){ if(!b.start){resetBucket(b,newStart,sec);return;} if(newStart!=b.start){flushBucket(b,oldPath);resetBucket(b,newStart,sec);} }

const MaxChannelConfig& maxCfg(const AppContext& c,uint8_t i){return i==0?c.config.max1:(i==1?c.config.max2:(i==2?c.config.max3:c.config.max4));} 
const MaxChannelReading& maxRead(const AppContext& c,uint8_t i){return c.maxReadings[i];}

String maxLabel(const AppContext& ctx,uint8_t idx){
  HeatSourceAssignment a; MaxChannel ch=(MaxChannel)idx;
  if(HeatSourceAssignments::getByChannel(ctx.heatSourceAssignments,ch,a) && a.assigned) return String(HeatSourceRoles::toLabel(a.role));
  return String("ADC")+String(idx+1);
}
String dsLabel(const AppContext& ctx,const Ds18b20DeviceInfo& d){
  for(uint8_t i=0;i<ctx.assignments.count;i++){
    const auto& a=ctx.assignments.items[i]; if(a.assigned && strcmp(a.addressText,d.addressText)==0) return String(SensorRoles::toLabel(a.role));
  }
  return String("DS18 ")+String(d.addressText);
}

void addAllSensors(const AppContext& ctx,Bucket& b5,Bucket& b30,Bucket& bd){
  for(uint8_t i=0;i<MAX_MAX31865_CHANNELS;i++){
    const auto& cfg=maxCfg(ctx,i); const auto& r=maxRead(ctx,i); if(!cfg.enabled||!r.valid)continue;
    char id[8];snprintf(id,sizeof(id),"max%u",i+1);String lab=maxLabel(ctx,i);addValue(b5,id,lab.c_str(),r.tempC);addValue(b30,id,lab.c_str(),r.tempC);addValue(bd,id,lab.c_str(),r.tempC);
  }
  for(uint8_t i=0;i<ctx.ds18b20.count;i++){
    const auto& d=ctx.ds18b20.devices[i]; if(!d.present||!d.lastValid)continue; String lab=dsLabel(ctx,d); addValue(b5,d.addressText,lab.c_str(),d.lastTempC); addValue(b30,d.addressText,lab.c_str(),d.lastTempC); addValue(bd,d.addressText,lab.c_str(),d.lastTempC);
  }
}

bool shownId(const String& id){ Pref* p=findPref(id.c_str(),false);return p&&p->show; }

bool parseTimestamp(const String& s,time_t& out){ int Y,M,D,h,m,sec; if(sscanf(s.c_str(),"%d-%d-%dT%d:%d:%d",&Y,&M,&D,&h,&m,&sec)!=6)return false; struct tm t={};t.tm_year=Y-1900;t.tm_mon=M-1;t.tm_mday=D;t.tm_hour=h;t.tm_min=m;t.tm_sec=sec;t.tm_isdst=-1;out=mktime(&t);return out>0; }

bool parseDateArg(const String& s,struct tm& t){int Y,M,D;if(sscanf(s.c_str(),"%d-%d-%d",&Y,&M,&D)!=3)return false;t={};t.tm_year=Y-1900;t.tm_mon=M-1;t.tm_mday=D;t.tm_hour=0;t.tm_isdst=-1;return true;}

void sendChunk(WebServer& server,const String& s){server.sendContent(s);yield();}

void streamTempRows(WebServer& server,const String& path,time_t from,time_t to,bool& firstPoint){
  File f=SD.open(path.c_str(),FILE_READ);if(!f)return; if(f.available())f.readStringUntil('\n');
  while(f.available()){
    String line=f.readStringUntil('\n');line.trim();if(!line.length())continue;
    int p1=line.indexOf(',');if(p1<0)continue;String ts=line.substring(0,p1);time_t ep=0;if(!parseTimestamp(ts,ep)||ep<from||ep>=to)continue;
    int p2=line.indexOf(',',p1+1); if(p2<0)continue; String type=line.substring(p1+1,p2); if(type!="temp")continue;
    int p3=line.indexOf(',',p2+1); if(p3<0)continue; String id=line.substring(p2+1,p3); if(!shownId(id))continue;
    int p4=line.indexOf(',',p3+1);if(p4<0)continue;String label=line.substring(p3+1,p4);
    int p5=line.indexOf(',',p4+1);int p6=line.indexOf(',',p5+1);int p7=line.indexOf(',',p6+1);
    if(p5<0||p6<0||p7<0)continue;
    String avg=line.substring(p4+1,p5), minv=line.substring(p5+1,p6), maxv=line.substring(p6+1,p7);
    String o=(firstPoint?"":",");firstPoint=false;o+="{\"t\":\""+ts+"\",\"id\":\""+jsonEscape(id)+"\",\"label\":\""+jsonEscape(label)+"\",\"avg\":"+avg+",\"min\":"+minv+",\"max\":"+maxv+"}";sendChunk(server,o);
  }
  f.close();
}

} // namespace

namespace History {
void begin(AppContext& ctx){(void)ctx;ensureDirs();loadPrefs();g_lastSampleMs=0;g_lastEnergyKWh=NAN;}
void process(AppContext& ctx){
  if(!g_enabled || !TimeService::valid() || !ctx.sdAvailable)return;
  uint32_t nowMs=millis();if(g_lastSampleMs && (uint32_t)(nowMs-g_lastSampleMs)<g_intervalMs)return;g_lastSampleMs=nowMs;
  time_t now=TimeService::nowEpoch();if(!now)return;
  time_t s5=floorLocal(now,300),s30=floorLocal(now,1800),sd=floorLocal(now,86400);
  String old5=dayPath(g_5m.start?g_5m.start:s5), old30=monthPath(g_30m.start?g_30m.start:s30), oldD=yearPath(g_day.start?g_day.start:sd);
  advanceBucket(g_5m,s5,300,old5);advanceBucket(g_30m,s30,1800,old30);advanceBucket(g_day,sd,86400,oldD);

  // DS18B20-FSM: History liest ausschliesslich den bereits publizierten
  // Inventory-Snapshot. Logging startet selbst keine Sensor-Conversion.
  addAllSensors(ctx,g_5m,g_30m,g_day);
  if(ctx.config.energyMeter.enabled && isfinite(ctx.energyMeter.totalEnergyKWh)){
    double cur=ctx.energyMeter.totalEnergyKWh;if(isfinite(g_lastEnergyKWh)){double d=(cur-g_lastEnergyKWh)*1000.0;if(d>=0 && d<100000){g_5m.energyWh+=d;g_30m.energyWh+=d;g_day.energyWh+=d;}}g_lastEnergyKWh=cur;
  } else g_lastEnergyKWh=NAN;
}
bool enabled(){return g_enabled;} void setEnabled(bool v){
  if(g_enabled==v)return;
  g_enabled=v;
  g_lastSampleMs=0;
  g_lastEnergyKWh=NAN;
  resetBucket(g_5m,0,300); resetBucket(g_30m,0,1800); resetBucket(g_day,0,86400);
} bool displayEnabled(const char* id){Pref* p=findPref(id,false);return p&&p->show;} void setDisplayEnabled(const char* id,bool v){Pref* p=findPref(id,true);if(p)p->show=v;} bool savePreferences(){return savePrefsInternal();}

String configJson(const AppContext& ctx){
  String j="{\"enabled\":"+String(g_enabled?"true":"false")+",\"intervalMs\":"+String(g_intervalMs)+",\"sensors\":[";bool first=true;
  for(uint8_t i=0;i<MAX_MAX31865_CHANNELS;i++){const auto& cfg=maxCfg(ctx,i);if(!cfg.enabled)continue;char id[8];snprintf(id,sizeof(id),"max%u",i+1);String lab=maxLabel(ctx,i);if(!first)j+=",";first=false;j+="{\"id\":\""+String(id)+"\",\"label\":\""+jsonEscape(lab)+"\",\"source\":\"MAX\",\"show\":"+String(displayEnabled(id)?"true":"false")+"}";}
  for(uint8_t i=0;i<ctx.ds18b20.count;i++){const auto& d=ctx.ds18b20.devices[i];if(!d.present)continue;String lab=dsLabel(ctx,d);if(!first)j+=",";first=false;j+="{\"id\":\""+String(d.addressText)+"\",\"label\":\""+jsonEscape(lab)+"\",\"source\":\"DS18B20\",\"show\":"+String(displayEnabled(d.addressText)?"true":"false")+"}";}
  j+="]}";return j;
}

void handleHistoryRequest(AppContext& ctx, WebServer& server){
  (void)ctx;
  String range=server.hasArg("range")?server.arg("range"):"day";String date=server.hasArg("date")?server.arg("date"):"";
  time_t now=TimeService::nowEpoch();struct tm base{};if(date.length()&&!parseDateArg(date,base)){server.send(400,"application/json","{\"error\":\"bad_date\"}");return;}if(!date.length()){localtime_r(&now,&base);base.tm_hour=0;base.tm_min=0;base.tm_sec=0;base.tm_isdst=-1;}
  time_t from=mktime(&base),to=from;String paths[3];uint8_t pathCount=0;
  if(range=="day"){to=from+86400;paths[pathCount++]=dayPath(from);} 
  else if(range=="week"){from-=6*86400;to=mktime(&base)+86400;struct tm a,b;localtime_r(&from,&a);localtime_r(&to,&b);char p[48];strftime(p,sizeof(p),"/logs/history/month/%Y-%m.csv",&a);paths[pathCount++]=p;char p2[48];strftime(p2,sizeof(p2),"/logs/history/month/%Y-%m.csv",&b);if(String(p2)!=String(p))paths[pathCount++]=p2;}
  else if(range=="month"){base.tm_mday=1;from=mktime(&base);struct tm next=base;next.tm_mon+=1;to=mktime(&next);paths[pathCount++]=monthPath(from);} 
  else if(range=="year"){base.tm_mon=0;base.tm_mday=1;from=mktime(&base);struct tm next=base;next.tm_year+=1;to=mktime(&next);paths[pathCount++]=yearPath(from);} 
  else {server.send(400,"application/json","{\"error\":\"bad_range\"}");return;}

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);server.send(200,"application/json","");
  sendChunk(server,"{\"range\":\""+range+"\",\"from\":\""+tsLocal(from)+"\",\"to\":\""+tsLocal(to)+"\",\"points\":[");
  bool fp=true,fe=true;double cumulative=0;
  // First pass temperature points only; function also sees energy, so buffer energy in second pass by resetting flags impossible. Use same pass but JSON needs energy after points.
  // To keep streaming bounded, first pass emits temperatures and ignores energy by local dummy flags/cumulative.
  for(uint8_t i=0;i<pathCount;i++) streamTempRows(server,paths[i],from,to,fp);
  sendChunk(server,"],\"energy\":[");
  // Second pass: suppress temperatures by temporarily relying on display mask? Dedicated lightweight parse below.
  for(uint8_t pi=0;pi<pathCount;pi++){
    File f=SD.open(paths[pi].c_str(),FILE_READ);if(!f)continue;if(f.available())f.readStringUntil('\n');
    while(f.available()){
      String line=f.readStringUntil('\n');line.trim();if(!line.length())continue;int p1=line.indexOf(',');int p2=line.indexOf(',',p1+1);if(p1<0||p2<0)continue;String ts=line.substring(0,p1),type=line.substring(p1+1,p2);if(type!="energy")continue;time_t ep;if(!parseTimestamp(ts,ep)||ep<from||ep>=to)continue;int last=line.lastIndexOf(',');if(last<0)continue;double wh=line.substring(last+1).toDouble();cumulative+=wh/1000.0;String o=(fe?"":",");fe=false;o+="{\"t\":\""+ts+"\",\"kwh\":"+String(cumulative,4)+"}";sendChunk(server,o);
    }f.close();
  }
  sendChunk(server,"]}");
}
}
