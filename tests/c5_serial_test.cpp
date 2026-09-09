#include "C5Serial.hpp"
#include <future>
#include <iostream>
#include <pty.h>

int main() {
    int master,slave;char path[128];
    if (openpty(&master,&slave,path,nullptr,nullptr)) return 1;
    try {
        C5Serial port(path);
        auto device=std::async(std::launch::async,[&] {
            char cmd[5];size_t got=0;
            while(got<sizeof(cmd)) {
                auto n=::read(master,cmd+got,sizeof(cmd)-got);
                if(n<=0) throw std::runtime_error("PTY read failed");
                got+=n;
            }
            if(std::string(cmd,5)!="INFO\n") throw std::runtime_error("bad command");
            const std::string response="C5SDR 1\r\n";
            for(char ch:response) if(::write(master,&ch,1)!=1) throw std::runtime_error("PTY write failed");
            const uint8_t binary[]={0,10,13,255};
            if(::write(master,binary,4)!=4) throw std::runtime_error("PTY binary write failed");
        });
        const auto deadline=C5Serial::Clock::now()+std::chrono::seconds(2);
        port.command("INFO",deadline);
        if(port.line(deadline)!="C5SDR 1") throw std::runtime_error("bad line");
        uint8_t data[4];port.read(data,4,deadline);
        if(data[0]!=0 || data[1]!=10 || data[2]!=13 || data[3]!=255) throw std::runtime_error("binary altered");
        device.get();
        bool timedOut=false;
        try { port.read(data,1,C5Serial::Clock::now()+std::chrono::milliseconds(20)); }
        catch(const std::runtime_error &e) { timedOut=std::string(e.what()).find("timeout")!=std::string::npos; }
        if(!timedOut) throw std::runtime_error("missing deadline");
        ::close(master);::close(slave);
        std::cout<<"C5 serial framing, binary transparency, deadline: OK\n";
        return 0;
    } catch(const std::exception &e) { std::cerr<<e.what()<<'\n';::close(master);::close(slave);return 1; }
}
