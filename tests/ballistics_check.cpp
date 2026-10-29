#include "rm_tracker/ballistics.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>

using dt46_vision::solve_projectile;
using State = std::array<double, 4>; // x, z, vx, vz

State derivative(const State& y, double drag) {
    const double loss = drag * std::hypot(y[2], y[3]);
    return {y[2], y[3], -loss*y[2], -9.81-loss*y[3]};
}
State add(const State& a, const State& b, double scale) {
    State y{};
    for (int i=0; i<4; ++i) y[i]=a[i]+scale*b[i];
    return y;
}
std::array<double, 2> reference(double range, double pitch, double speed, double drag) {
    State y{0,0,speed*std::cos(pitch),speed*std::sin(pitch)};
    constexpr double dt=.0001;
    for (int i=0; i<30000; ++i) {
        const auto a=derivative(y,drag), b=derivative(add(y,a,dt/2),drag);
        const auto c=derivative(add(y,b,dt/2),drag), d=derivative(add(y,c,dt),drag);
        State next=y;
        for(int j=0;j<4;++j)next[j]+=dt/6*(a[j]+2*b[j]+2*c[j]+d[j]);
        if(next[0]>=range){
            const double f=(range-y[0])/(next[0]-y[0]);
            return {y[1]+f*(next[1]-y[1]),(i+f)*dt};
        }
        y=next;
    }
    throw std::runtime_error("reference did not reach range");
}

int main() {
    double max_height=0,max_time=0; int count=0;
    for(double drag:{0.,.019})for(int iv=20;iv<=30;++iv)for(int ir=0;ir<49;++ir)for(int iz=-14;iz<=14;++iz){
        const double range=ir==0?.2:ir==1?.3:.5+(ir-2)*.25,height=iz*.25,speed=iv;
        const auto s=solve_projectile(range,height,speed,drag);
        if(!s.valid)throw std::runtime_error("valid input rejected");
        const auto actual=reference(range,s.pitch,speed,drag);
        max_height=std::max(max_height,std::abs(actual[0]-height));
        max_time=std::max(max_time,std::abs(actual[1]-s.flight_time));
        if(std::abs(actual[0]-height)>(drag==0?1e-6:.01)||std::abs(actual[1]-s.flight_time)>.0006)
            throw std::runtime_error("ballistic accuracy regression");
        ++count;
    }
    const double nan=std::numeric_limits<double>::quiet_NaN();
    const double inf=std::numeric_limits<double>::infinity();
    for(const auto args:std::array<State,10>{{{.1,0,25,.019},{12.1,0,25,.019},{3,3.6,25,.019},
        {3,-3.6,25,.019},{3,0,0,.019},{3,0,25,-.019},{3,0,nan,.019},{3,0,25,inf},{nan,0,25,.019},{12,3.5,5,.019}}})
        if(solve_projectile(args[0],args[1],args[2],args[3]).valid)throw std::runtime_error("invalid input accepted");
    volatile double sink=0;
    const auto begin=std::chrono::steady_clock::now();
    for(int i=0;i<1000000;++i){
        const auto s=solve_projectile(3+(i%900)*.01,.1,25+(i%3)*.3,.019);
        sink=s.pitch+s.flight_time+double(s.valid);
    }
    const double ns=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-begin).count()/1000000;
    std::cout<<"cases="<<count<<" max_height_error_m="<<max_height<<" max_time_error_s="<<max_time<<" solve_ns="<<ns<<'\n';
    return 0;
}
