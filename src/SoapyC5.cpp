#include "SoapyC5.hpp"
#include "C5Serial.hpp"
#include "C5IQ.hpp"
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Logger.hpp>
#include <zlib.h>
#include <cmath>
#include <complex>
#include <memory>
#include <mutex>
#include <sstream>

namespace {
constexpr size_t capacity=16380;
struct Burst {
    int direction;
    std::string format;
    bool active=false;
    size_t requested=0, offset=0;
    std::vector<uint32_t> words;
};
class C5Device final : public SoapySDR::Device {
    C5Serial port;
    mutable std::mutex mutex;
    std::unique_ptr<Burst> streams[2];
    double rates[2]={1000000,80000000};
    double frequency=2412000000;
    bool failed=false;
    unsigned txRepeats=1;
    unsigned wireBits=10;
    static void channel(int dir,size_t ch) {
        if((dir!=SOAPY_SDR_RX && dir!=SOAPY_SDR_TX)||ch)throw std::runtime_error("C5: invalid channel");
    }
    static C5Serial::Deadline deadline(long us=3000000) {
        return C5Serial::Clock::now()+std::chrono::microseconds(std::max(0L,us));
    }
    Burst &get(SoapySDR::Stream *s) const {
        for(auto &p:streams)if(p.get()==reinterpret_cast<Burst *>(s) && p)return *p;
        throw std::runtime_error("C5: invalid stream");
    }
    bool active() const {return (streams[0]&&streams[0]->active)||(streams[1]&&streams[1]->active);}
    void idle() const {
        if(failed)throw std::runtime_error("C5: transport failed; reopen device after firmware timeout");
        if(active())throw std::runtime_error("C5: deactivate stream before changing RF settings");
    }
public:
    explicit C5Device(const SoapySDR::Kwargs &args):port(args.at("serial")) {
        auto end=deadline();
        const auto token=std::to_string(C5Serial::Clock::now().time_since_epoch().count());
        port.command("\nSYNC "+token,end);
        const std::string marker="SYNC "+token+"\n";
        std::string window;
        for(size_t bytes=0;window!=marker;bytes++) {
            if(bytes>=70000)throw std::runtime_error("C5: synchronization failed");
            char c;port.read(&c,1,end);window+=c;
            if(window.size()>marker.size())window.erase(0,1);
        }
        port.command("INFO",end);
        auto identity=port.line(end);
        // A ROM-to-app transition can leave a partial command in the FIFO.
        if(identity=="ERR command" || identity=="ERR command_length") {
            port.command("INFO",end);identity=port.line(end);
        }
        if(identity!="C5SDR 6 burst 16380")throw std::runtime_error("C5: protocol-6 firmware required");
        port.command("FREQ 2412",end);
        if(port.line(end)!="OK")throw std::runtime_error("C5: initial tune failed");
    }
    std::string getDriverKey() const override{return "espsdr";}
    std::string getHardwareKey() const override{return "ESP32-C5";}
    SoapySDR::Kwargs getHardwareInfo() const override {
        return {{"transport","USB Serial/JTAG"},{"mode","finite burst, half duplex"},{"protocol","6"}};
    }
    SoapySDR::ArgInfoList getSettingInfo() const override {
        SoapySDR::ArgInfo a;a.key="TX_REPEATS";a.value="1";
        a.name="TX buffer repetitions";a.type=SoapySDR::ArgInfo::INT;
        a.description="Repeat each uploaded TX buffer 1–255 times; total playback must be at most 100 ms.";
        a.range=SoapySDR::Range(1,255,1);
        SoapySDR::ArgInfo w;w.key="WIRE_BITS";w.value="10";w.name="USB bits per I and Q";
        w.type=SoapySDR::ArgInfo::INT;w.options={"10","8"};
        w.description="10 is lossless; 8 discards two low bits per component. Applies to RX and TX while inactive.";
        return {a,w};
    }
    void writeSetting(const std::string &key,const std::string &value) override {
        std::lock_guard<std::mutex> lock(mutex);idle();
        if(key=="WIRE_BITS") {
            if(value!="8" && value!="10")throw std::runtime_error("C5: WIRE_BITS must be 8 or 10");
            wireBits=unsigned(std::stoul(value));return;
        }
        if(key!="TX_REPEATS")throw std::runtime_error("C5: unknown setting");
        size_t used=0;unsigned long n=std::stoul(value,&used);
        if(used!=value.size()||n<1||n>255)throw std::runtime_error("C5: TX_REPEATS must be 1–255");
        txRepeats=unsigned(n);
    }
    std::string readSetting(const std::string &key) const override {
        std::lock_guard<std::mutex> lock(mutex);
        if(key=="WIRE_BITS")return std::to_string(wireBits);
        if(key!="TX_REPEATS")throw std::runtime_error("C5: unknown setting");
        return std::to_string(txRepeats);
    }
    size_t getNumChannels(int) const override{return 1;}
    bool getFullDuplex(int,size_t) const override{return false;}
    std::vector<std::string> getStreamFormats(int,size_t) const override{return {SOAPY_SDR_CF32,SOAPY_SDR_CS16};}
    std::string getNativeStreamFormat(int,size_t,double &scale) const override{scale=32768;return SOAPY_SDR_CS16;}
    std::vector<std::string> listAntennas(int,size_t) const override{return {"RF"};}
    std::string getAntenna(int,size_t) const override{return "RF";}
    void setAntenna(int,size_t,const std::string &name) override {
        if(name!="RF")throw std::runtime_error("C5: antenna must be RF");
    }
    std::vector<std::string> listFrequencies(int,size_t) const override{return {"RF"};}
    SoapySDR::RangeList getFrequencyRange(int,size_t) const override{return {{2100e6,2700e6,1e6},{4800e6,6000e6,1e6}};}
    SoapySDR::RangeList getFrequencyRange(int d,size_t c,const std::string &) const override{return getFrequencyRange(d,c);}
    void setFrequency(int d,size_t c,double hz,const SoapySDR::Kwargs &) override {
        channel(d,c);std::lock_guard<std::mutex> lock(mutex);idle();
        if(!std::isfinite(hz)||!((hz>=2100e6&&hz<=2700e6)||(hz>=4800e6&&hz<=6000e6)))throw std::runtime_error("C5: frequency outside 2100–2700 / 4800–6000 MHz");
        auto mhz=std::lround(hz/1e6);auto end=deadline();port.command("FREQ "+std::to_string(mhz),end);
        if(port.line(end)!="OK")throw std::runtime_error("C5: tune failed");
        frequency=mhz*1e6;
    }
    void setFrequency(int d,size_t c,const std::string &name,double hz,const SoapySDR::Kwargs &a) override {
        if(name!="RF")throw std::runtime_error("C5: unknown frequency component");
        setFrequency(d,c,hz,a);
    }
    double getFrequency(int,size_t) const override{std::lock_guard<std::mutex> lock(mutex);return frequency;}
    double getFrequency(int d,size_t c,const std::string &) const override{return getFrequency(d,c);}
    std::vector<double> listSampleRates(int d,size_t c) const override {
        channel(d,c);
        if(d==SOAPY_SDR_TX)return {250000,500000,1000000,2000000,3000000,4000000,6000000,40000000,80000000};
        return {4000000,8000000,10000000,20000000,40000000,80000000};
    }
    SoapySDR::RangeList getSampleRateRange(int d,size_t c) const override {
        SoapySDR::RangeList r;for(double v:listSampleRates(d,c))r.emplace_back(v,v);return r;
    }
    void setSampleRate(int d,size_t c,double rate) override {
        channel(d,c);std::lock_guard<std::mutex> lock(mutex);idle();auto values=listSampleRates(d,c);
        if(std::find(values.begin(),values.end(),rate)==values.end())throw std::runtime_error("C5: unsupported sample rate");
        rates[d]=rate;
    }
    double getSampleRate(int d,size_t c) const override {channel(d,c);std::lock_guard<std::mutex> lock(mutex);return rates[d];}
    SoapySDR::Stream *setupStream(int d,const std::string &format,const std::vector<size_t> &channels,const SoapySDR::Kwargs &) override {
        channel(d,0);std::lock_guard<std::mutex> lock(mutex);
        if(!channels.empty()&&channels!=std::vector<size_t>{0})throw std::runtime_error("C5: one channel only");
        if(format!=SOAPY_SDR_CF32 && format!=SOAPY_SDR_CS16)throw std::runtime_error("C5: unsupported format");
        if(streams[d])throw std::runtime_error("C5: stream already exists");
        streams[d]=std::make_unique<Burst>();streams[d]->direction=d;streams[d]->format=format;
        return reinterpret_cast<SoapySDR::Stream *>(streams[d].get());
    }
    void closeStream(SoapySDR::Stream *s) override {std::lock_guard<std::mutex> lock(mutex);int d=get(s).direction;streams[d].reset();}
    size_t getStreamMTU(SoapySDR::Stream *) const override{return capacity;}
    int activateStream(SoapySDR::Stream *s,int flags,long long,size_t n) override {
        std::lock_guard<std::mutex> lock(mutex);auto &b=get(s);
        if(failed)return SOAPY_SDR_STREAM_ERROR;
        if(flags || (b.direction==SOAPY_SDR_RX && (n<256||n>capacity)) || n>capacity)return SOAPY_SDR_NOT_SUPPORTED;
        if(active())return SOAPY_SDR_STREAM_ERROR;
        b.active=true;b.requested=n;b.offset=0;b.words.clear();return 0;
    }
    int deactivateStream(SoapySDR::Stream *s,int flags,long long) override {
        std::lock_guard<std::mutex> lock(mutex);if(flags)return SOAPY_SDR_NOT_SUPPORTED;
        auto &b=get(s);b.active=false;b.words.clear();return 0;
    }
    int readStream(SoapySDR::Stream *s,void *const *buffs,size_t n,int &flags,long long &timeNs,long timeoutUs) override {
        std::lock_guard<std::mutex> lock(mutex);auto &b=get(s);flags=0;timeNs=0;
        if(failed||b.direction!=SOAPY_SDR_RX)return SOAPY_SDR_STREAM_ERROR;
        if(!b.active)return SOAPY_SDR_TIMEOUT;
        if(n==0)return 0;
        try {
            if(b.words.empty()) {
                auto end=deadline(timeoutUs);unsigned div=0;
                const double clocks[]={80000000,40000000,20000000,10000000,8000000,4000000};
                while(clocks[div]!=rates[SOAPY_SDR_RX])++div;
                port.command(std::string(wireBits==8?"CAP16 ":"CAP20 ")+std::to_string(b.requested)+" "+std::to_string(div),end);
                std::istringstream h(port.line(end));std::string tag;size_t count;uint32_t crc,us;
                if(!(h>>tag>>count>>std::hex>>crc>>std::dec>>us)||tag!="DATA"||count!=b.requested)throw std::runtime_error("C5: invalid capture header");
                std::vector<uint8_t> wire(wireBits==8?count*2:(count*20+7)/8);port.read(wire.data(),wire.size(),end);
                if(crc32(0,wire.data(),wire.size())!=crc)throw std::runtime_error("C5: capture CRC mismatch");
                b.words=wireBits==8?c5UnpackIQ8(wire,count):c5UnpackIQ(wire,count);
            }
            n=std::min(n,b.words.size()-b.offset);
            for(size_t j=0;j<n;j++) {
                auto w=b.words[b.offset+j];int i=w&1023,q=(w>>10)&1023;
                if(i>=512)i-=1024;
                if(q>=512)q-=1024;
                q=-q;
                if(b.format==SOAPY_SDR_CF32)static_cast<std::complex<float> *>(buffs[0])[j]={i/512.0f,q/512.0f};
                else {auto p=static_cast<int16_t *>(buffs[0]);p[2*j]=i*64;p[2*j+1]=std::min(32767,q*64);}
            }
            b.offset+=n;if(b.offset==b.words.size()){flags|=SOAPY_SDR_END_BURST;b.active=false;}
            return int(n);
        }catch(const std::exception &e){failed=true;b.active=false;SoapySDR::log(SOAPY_SDR_ERROR,e.what());return SOAPY_SDR_STREAM_ERROR;}
    }
    int writeStream(SoapySDR::Stream *s,const void *const *buffs,size_t n,int &flags,long long,long timeoutUs) override {
        std::lock_guard<std::mutex> lock(mutex);auto &b=get(s);
        if(failed||!b.active||b.direction!=SOAPY_SDR_TX)return SOAPY_SDR_STREAM_ERROR;
        if(flags!=SOAPY_SDR_END_BURST || !n || n>capacity || (b.requested&&n!=b.requested))return SOAPY_SDR_NOT_SUPPORTED;
        if(n*txRepeats>size_t(rates[SOAPY_SDR_TX]/10))return SOAPY_SDR_NOT_SUPPORTED;
        std::vector<uint32_t> words(n);
        for(size_t j=0;j<n;j++) {
            float i,q;
            if(b.format==SOAPY_SDR_CF32){auto v=static_cast<const std::complex<float> *>(buffs[0])[j];i=v.real()*512;q=v.imag()*512;}
            else {auto p=static_cast<const int16_t *>(buffs[0]);i=p[2*j]/64.0f;q=p[2*j+1]/64.0f;}
            if(!std::isfinite(i)||!std::isfinite(q))return SOAPY_SDR_STREAM_ERROR;
            uint32_t w=(uint32_t(int(std::round(std::clamp(i,-512.0f,511.0f))))&1023)|((uint32_t(int(std::round(std::clamp(q,-512.0f,511.0f))))&1023)<<10);
            words[j]=w;
        }
        auto wire=wireBits==8?c5PackIQ8(words):c5PackIQ(words);
        try {
            auto end=deadline(timeoutUs);std::ostringstream cmd;
            cmd<<(txRepeats==1?(wireBits==8?"TX16 ":"TX20 "):(wireBits==8?"LOOP16 ":"LOOP20 "))<<n<<" "<<unsigned(rates[SOAPY_SDR_TX])<<" ";
            if(txRepeats!=1)cmd<<txRepeats<<" ";
            cmd<<std::hex<<crc32(0,wire.data(),wire.size());
            port.command(cmd.str(),end);
            if(port.line(end)!="READY")throw std::runtime_error("C5: TX rejected");
            port.write(wire.data(),wire.size(),end);
            std::istringstream h(port.line(end));std::string tag;size_t sent;uint32_t cycles,late;
            if(!(h>>tag>>sent>>cycles>>late)||tag!="SENT"||sent!=n*txRepeats)throw std::runtime_error("C5: TX failed");
            b.active=false;
            if(late>=240000000u/unsigned(rates[SOAPY_SDR_TX]))return SOAPY_SDR_UNDERFLOW;
            return int(n);
        }catch(const std::exception &e){failed=true;b.active=false;SoapySDR::log(SOAPY_SDR_ERROR,e.what());return SOAPY_SDR_STREAM_ERROR;}
    }
};
}
SoapySDR::Device *makeC5(const SoapySDR::Kwargs &args){return new C5Device(args);}
SoapySDR::KwargsList findC5(const SoapySDR::Kwargs &args) {
    try {C5Device probe(args);auto result=args;result["driver"]="espsdr";result["label"]="ESP32-C5 USB burst SDR";return {result};}
    catch(const std::exception &e){SoapySDR::log(SOAPY_SDR_DEBUG,e.what());return {};}
}
