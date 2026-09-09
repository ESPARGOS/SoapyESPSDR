#include "SoapyC5.hpp"
#include "C5IQ.hpp"
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>
#include <zlib.h>
#include <complex>
#include <future>
#include <iostream>
#include <memory>
#include <pty.h>
#include <sstream>
#include <unistd.h>

static void require(bool ok,const char *what){if(!ok)throw std::runtime_error(what);}
int main() {
    // Independent byte fixtures exercise nibble sharing and odd tails.
    require(c5PackIQ({0x54321,0xabcde,0xfffff})==std::vector<uint8_t>({0x21,0x43,0xe5,0xcd,0xab,0xff,0xff,0x0f}),"packed IQ fixture");
    require(c5UnpackIQ({0x21,0x43,0xe5,0xcd,0xab,0xff,0xff,0x0f},3)==std::vector<uint32_t>({0x54321,0xabcde,0xfffff}),"unpacked IQ fixture");
    require(c5PackIQ8({0,0x803ff,0xfffff})==std::vector<uint8_t>({0,0,255,128,255,255}),"IQ8 signed fixture");
    std::vector<uint32_t> all(1u<<20);
    for(size_t j=0;j<all.size();j++)all[j]=j;
    auto restored=c5UnpackIQ8(c5PackIQ8(all),all.size());
    for(size_t j=0;j<all.size();j++)require(restored[j]==(j&0xff3fc),"IQ8 exhaustive conversion");
    int master,slave;char path[128];
    if(openpty(&master,&slave,path,nullptr,nullptr))return 1;
    auto emulator=std::async(std::launch::async,[&]{
        auto line=[&]{std::string s;char c;while(::read(master,&c,1)==1){if(c=='\n')return s;s+=c;}throw std::runtime_error("PTY closed");};
        auto send=[&](const std::string &s){require(::write(master,s.data(),s.size())==ssize_t(s.size()),"emulator write");};
        require(line().empty(),"synchronization boundary");
        auto sync=line();require(sync.rfind("SYNC ",0)==0,"synchronization request");
        send("old data\nC5SDR 6 burst 16380\n"+sync+"\n");
        require(line()=="INFO","identity query");send("C5SDR 6 burst 16380\n");
        require(line()=="FREQ 2412","initial tune");send("OK\n");
        std::string wire(256*5/2,'\0');
        // I=-512, Q=-512 -> canonical RX (-1,+1). Includes binary newline.
        wire[0]=0;wire[1]=2;wire[2]=char(0xa8);
        for(unsigned attempt=0;attempt<2;attempt++) {
            require(line()=="CAP20 256 0","capture query");
            std::ostringstream h;h<<"DATA 256 "<<std::hex<<(crc32(0,reinterpret_cast<const Bytef *>(wire.data()),wire.size())^attempt)<<" 2000\n";
            send(h.str());send(wire);
        }
    });
    try {
        std::unique_ptr<SoapySDR::Device> d(makeC5({{"serial",path}}));
        auto ranges=d->getFrequencyRange(SOAPY_SDR_RX,0);
        require(ranges.size()==2 && ranges[0].minimum()==2100e6 && ranges[0].maximum()==2700e6 &&
                ranges[1].minimum()==4800e6 && ranges[1].maximum()==6000e6,"dual band ranges");
        for(double frequency : {2099e6,2701e6,4000e6,4799e6,6001e6}) {
            bool rejected=false;
            try {d->setFrequency(SOAPY_SDR_RX,0,frequency);}catch(const std::exception &){rejected=true;}
            require(rejected,"unsupported/gap frequency accepted");
        }
        auto s=d->setupStream(SOAPY_SDR_RX,SOAPY_SDR_CF32);
        require(d->activateStream(s,0,0,256)==0,"activation");
        std::complex<float> data[256];void *buffs[]={data};int flags=0;long long ns=0;
        require(d->readStream(s,buffs,128,flags,ns,1000000)==128,"partial read");
        require(data[0]==std::complex<float>(-1,1),"signed IQ orientation");
        require(data[1]==std::complex<float>(10.0f/512,0),"binary transparency");
        require(flags==0,"premature end burst");
        require(d->readStream(s,buffs,128,flags,ns,1000000)==128,"remaining read");
        require(flags==SOAPY_SDR_END_BURST,"missing end burst");
        require(d->activateStream(s,0,0,256)==0,"second activation");
        require(d->readStream(s,buffs,256,flags,ns,1000000)==SOAPY_SDR_STREAM_ERROR,"corrupt CRC accepted");
        require(d->activateStream(s,0,0,256)==SOAPY_SDR_STREAM_ERROR,"failed transport reused");
        d->closeStream(s);emulator.get();
        ::close(master);::close(slave);std::cout<<"C5 signed IQ, partial burst framing, CRC rejection: OK\n";
        return 0;
    }catch(const std::exception &e){std::cerr<<e.what()<<'\n';::close(slave);::close(master);return 1;}
}
