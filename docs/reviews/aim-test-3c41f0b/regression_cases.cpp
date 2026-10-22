// Appended inside the generated harness main; all tested logic comes from the working tree.
    Light left{{100,106},{101,94},100,100};
    Light right{{128,106},{127,94},127,100};
    assert(slopes_match(left,right));
    const double angles[]={-30,-20,-4,0,4,20,30};
    for(double a:angles) {
        for(double b:angles) {
            Light one{{100,106},{100+12*std::sin(a*DEG2RAD),106-12*std::cos(a*DEG2RAD)},100,100};
            Light two{{128,106},{128+12*std::sin(b*DEG2RAD),106-12*std::cos(b*DEG2RAD)},128,100};
            assert(slopes_match(one,two)==(std::abs(a-b)<=35));
        }
    }
    right.cy=150;
    assert(!slopes_match(left,right));
    std::cout << "R01 PASS: opposite tilts, 49 angle pairs, and steep connecting line\n";

    Tracker history;
    history.obs_center_hist_={{0,5,0},{.5,5,.4}};
    history.move_clock_=.5;
    assert(!history.center_motion_is_slow());
    history.spin_aim_translate_=true;
    for(int i=0;i<200;++i) {
        history.move_clock_+=.01;
        history.update_spin_aim_mode(.01);
    }
    assert(history.center_motion_is_slow());
    assert(history.obs_center_hist_.empty());
    assert(!history.spin_aim_translate_);
    // A single fresh observation cannot revive the expired movement window.
    history.obs_center_hist_.push_back({history.move_clock_,5,0});
    assert(history.center_motion_is_slow());
    // Fresh continuous translation must still enter the mode after hysteresis.
    for(int i=1;i<=110;++i) {
        history.move_clock_+=.01;
        history.obs_center_hist_.push_back({history.move_clock_,5,i*.01});
        history.update_spin_aim_mode(.01);
    }
    assert(history.spin_aim_translate_);
    history.spin_=false;
    history.update_spin_aim_mode(.01);
    assert(!history.spin_aim_translate_);
    history.spin_=true;
    history.obs_center_hist_={{10,5,0},{10.5,5,0}};
    history.move_clock_=10.5;
    assert(history.center_motion_is_slow());
    std::cout << "R04 PASS: expiry, mode exit, insufficient samples, recovery, and stationary history\n";

    const double radius=.24;
    for(double bearing:{0.0,179.0*DEG2RAD,-179.0*DEG2RAD}) {
        for(double offset:{-30.0*DEG2RAD,30.0*DEG2RAD}) {
            Tracker yaw;
            const double expected=bearing+offset;
            const double px=5*std::cos(bearing), py=5*std::sin(bearing);
            yaw.target_state_(0)=px+radius*std::cos(expected);
            yaw.target_state_(2)=py+radius*std::sin(expected);
            yaw.target_state_(6)=expected;
            yaw.target_state_(8)=radius;
            yaw.ekf_.state=yaw.target_state_;
            for(double observed:{bearing-offset,expected}) {
                Armor raw(0,px,py,0,normalize_angle(observed));
                auto matched=yaw.match_observation({raw});
                assert(matched.has_value());
                yaw.apply_armor_measurement(*matched);
                assert(std::abs(shortest_angular_distance(expected,yaw.ekf_.observed(3)))<1e-9);
                assert(raw.yaw==normalize_angle(observed));
            }
        }
    }
    std::cout << "R03 PASS: association to observation update, mirrored/unmirrored yaw, and +/-pi crossing\n";

    Tracker spin;
    spin.build_ballistic_lut();
    spin.target_state_(0)=5+radius;
    spin.target_state_(8)=radius;
    spin.another_r_=radius;
    spin.plate_(0)=5;
    const double arrival=spin.lut_lookup_tof(5,0)+spin.system_delay;
    // Holding the chassis still must not use residual translation velocity.
    spin.target_state_(1)=2;
    spin.target_state_(3)=1;
    for(double speed:{-4.0,4.0}) {
        spin.spin_front_plate_i_=0;
        spin.target_state_(6)=0;
        spin.target_state_(7)=speed;
        Armor target=spin.build_target(.01);
        assert(std::abs(target.pos(0)-5)<1e-9 && std::abs(target.pos(1))<1e-9);
        assert(target.target_to_armor_dist>.059);
        assert(!std::get<2>(spin.can_fire(target,{0,0,true})));

        // The adjacent plate, not the currently observed plate, faces the camera on arrival.
        spin.target_state_(6)=std::copysign(CV_PI/2,speed)-speed*arrival;
        spin.another_r_=.20;
        spin.dz_=.04;
        target=spin.build_target(.01);
        assert(target.index%2==1);
        assert(std::abs(target.pos(0)-5.04)<1e-9 && std::abs(target.pos(2)-.04)<1e-9);
        assert(std::get<2>(spin.can_fire(target,{0,0,true})));
        spin.another_r_=radius;
        spin.dz_=0;
    }
    // Scan all phases in both directions: the independent oracle is a four-plate arrival window.
    spin.target_state_(1)=0;
    spin.target_state_(3)=0;
    int open_windows=0;
    for(double bearing:{0.0,179.0*DEG2RAD,-179.0*DEG2RAD}) {
        spin.target_state_(0)=(5+radius)*std::cos(bearing);
        spin.target_state_(2)=(5+radius)*std::sin(bearing);
        for(double speed:{-4.0,4.0}) {
            spin.target_state_(7)=speed;
            for(int phase=-180;phase<180;++phase) {
                spin.target_state_(6)=bearing+phase*DEG2RAD;
                Armor target=spin.build_target(.01);
                const double arrival_phase=phase*DEG2RAD+speed*arrival;
                const double nearest=std::abs(std::remainder(arrival_phase,CV_PI/2));
                const bool expected=radius*nearest<.059;
                const bool actual=std::get<2>(spin.can_fire(target,{0,0,true}));
                assert(actual==expected);
                open_windows+=actual;
            }
        }
    }
    assert(open_windows>0);
    // No valid center: follow observations but do not open the firing gate.
    spin.target_state_(0)=spin.target_state_(2)=0;
    spin.last_plate_yaw_=0;
    assert(!std::get<2>(spin.can_fire(spin.build_target(.01),{0,0,true})));
    // Translation mode still advances the center, with controller delay applied only to aiming.
    spin.target_state_(0)=5+radius;
    spin.target_state_(3)=1;
    spin.spin_aim_translate_=true;
    spin.move_clock_=.5;
    spin.obs_center_hist_={{0,5,0},{.5,5,.4}};
    const double aim_time=arrival+spin.controller_delay_;
    const double norm=std::hypot(5+radius,aim_time);
    Armor moving=spin.build_target(.01);
    assert(std::abs(moving.pos(0)-(5+radius)*(1-radius/norm))<1e-9);
    assert(std::abs(moving.pos(1)-aim_time*(1-radius/norm))<1e-9);
    // Ordinary plate leading remains active when spin is off.
    spin.spin_=false;
    spin.lead_vel_(1)=1;
    Armor ordinary=spin.build_target(.01);
    assert(std::abs(ordinary.pos(1)-aim_time)<1e-9);
    std::cout << "R02 PASS: old counterexample, future facing plate, alternating radius/height, 2160 phases, translation, and fallback\n";
    std::cout << "All four regression groups passed; ROS/camera/real-EKF integration remains unverified.\n";
}
