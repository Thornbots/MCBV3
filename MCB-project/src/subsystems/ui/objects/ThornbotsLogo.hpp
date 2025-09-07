#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/AtomicGraphicsObjects.hpp"

using namespace subsystems;

class ThornbotsLogo : public GraphicsContainer {
public:
    // 5 batches of 7 plus 1
    ThornbotsLogo() {
        // not sure what the order needs to be to make it layer properly
        // left on bottom, then mid, then right, then border on top
        addGraphicsObject(&border0);
        addGraphicsObject(&border1);
        addGraphicsObject(&border2);
        addGraphicsObject(&border3);
        addGraphicsObject(&border4);
        addGraphicsObject(&border5);
        addGraphicsObject(&border6);
        addGraphicsObject(&border7);
        addGraphicsObject(&border8);
        addGraphicsObject(&border9);
        addGraphicsObject(&border10);
        addGraphicsObject(&border11);
        addGraphicsObject(&border12);
        addGraphicsObject(&border13);
        addGraphicsObject(&border14);
        addGraphicsObject(&border15);
        addGraphicsObject(&border16);
        addGraphicsObject(&border17);
        addGraphicsObject(&border18);
        addGraphicsObject(&border19);
        addGraphicsObject(&border20);
        addGraphicsObject(&border21);
        addGraphicsObject(&border22);
        addGraphicsObject(&border23);

        
        addGraphicsObject(&right0);
        addGraphicsObject(&right1);
        addGraphicsObject(&right2);
        addGraphicsObject(&right3);
        addGraphicsObject(&right4);
        addGraphicsObject(&right5);
        addGraphicsObject(&right6);

        
        addGraphicsObject(&mid0);
        addGraphicsObject(&mid1);


        addGraphicsObject(&left0);
        addGraphicsObject(&left1);
        addGraphicsObject(&left2);
    }

    // no update, just draw once and not change
private:

    static constexpr uint16_t POSITION_X = 1250;  
    static constexpr uint16_t POSITION_Y = 810;   
    
    static constexpr uint16_t SIZE = 80;

    // left part of the rose
    Line left0{UISubsystem::Color::BLACK, (uint16_t) (0.2408500590318772*SIZE+POSITION_X), (uint16_t) (0.2845336481700118*SIZE+POSITION_Y), (uint16_t) (0.1959858323494687*SIZE+POSITION_X), (uint16_t) (0.71900826446281*SIZE+POSITION_Y), (uint16_t) (0.2361275088547816*SIZE)};
    Arc left1{UISubsystem::Color::BLACK, (uint16_t) (0.1381345926800472*SIZE+POSITION_X), (uint16_t) (0.3730814639905549*SIZE+POSITION_Y), (uint16_t) (0.5395513577331759*SIZE), (uint16_t) (0.7851239669421488*SIZE), 33, 340, (uint16_t) (0.1770956316410862*SIZE)};
    Arc left2{UISubsystem::Color::BLACK, (uint16_t) (0.2715466351829988*SIZE+POSITION_X), (uint16_t) (0.3955135773317591*SIZE+POSITION_Y), (uint16_t) (0.2597402597402597*SIZE), (uint16_t) (0.3707201889020071*SIZE), 270, 180, (uint16_t) (0.1180637544273908*SIZE)};

    // elephant
    Arc mid0{UISubsystem::Color::RED_AND_BLUE, (uint16_t) (0.5572609208972845*SIZE+POSITION_X), (uint16_t) (0.45454545454545453*SIZE+POSITION_Y), (uint16_t) (0.7142857142857143*SIZE), (uint16_t) (0.6765053128689492*SIZE), 33, 316, (uint16_t) (0.29515938606847697*SIZE)};
    Line mid1{UISubsystem::Color::RED_AND_BLUE, (uint16_t) (0.5100354191263282*SIZE+POSITION_X), (uint16_t) (0.14049586776859505*SIZE+POSITION_Y), (uint16_t) (0.5100354191263282*SIZE+POSITION_X), (uint16_t) (0.7579693034238488*SIZE+POSITION_Y), (uint16_t) (0.4722550177095632*SIZE)};

    // right part of the rose
    Arc right0{UISubsystem::Color::BLACK, (uint16_t) (0.8866587957497049*SIZE+POSITION_X), (uint16_t) (0.37662337662337664*SIZE+POSITION_Y), (uint16_t) (0.5395513577331759*SIZE), (uint16_t) (0.7851239669421488*SIZE), 15, 320, (uint16_t) (0.1770956316410862*SIZE)};
    Arc right1{UISubsystem::Color::BLACK, (uint16_t) (0.5147579693034239*SIZE+POSITION_X), (uint16_t) (0.29515938606847697*SIZE+POSITION_Y), (uint16_t) (0.51357733175915*SIZE), (uint16_t) (0.5324675324675324*SIZE), 238, 140, (uint16_t) (0.03541912632821724*SIZE)};
    Arc right2{UISubsystem::Color::BLACK, (uint16_t) (0.3837072018890201*SIZE+POSITION_X), (uint16_t) (0.30932703659976385*SIZE+POSITION_Y), (uint16_t) (0.29161747343565525*SIZE), (uint16_t) (0.4332939787485242*SIZE), 210, 140, (uint16_t) (0.0590318772136954*SIZE)};
    Line right3{UISubsystem::Color::BLACK, (uint16_t) (0.4946871310507674*SIZE+POSITION_X), (uint16_t) (0.047225501770956316*SIZE+POSITION_Y), (uint16_t) (0.6741440377804014*SIZE+POSITION_X), (uint16_t) (0.5726092089728453*SIZE+POSITION_Y), (uint16_t) (0.21251475796930341*SIZE)};
    Line right4{UISubsystem::Color::BLACK, (uint16_t) (0.6552538370720189*SIZE+POSITION_X), (uint16_t) (0.5289256198347108*SIZE+POSITION_Y), (uint16_t) (0.7579693034238488*SIZE+POSITION_X), (uint16_t) (0.6611570247933884*SIZE+POSITION_Y), (uint16_t) (0.21251475796930341*SIZE)};
    Line right5{UISubsystem::Color::BLACK, (uint16_t) (0.7591499409681228*SIZE+POSITION_X), (uint16_t) (0.28689492325855964*SIZE+POSITION_Y), (uint16_t) (0.8288075560802833*SIZE+POSITION_X), (uint16_t) (0.731995277449823*SIZE+POSITION_Y), (uint16_t) (0.21251475796930341*SIZE)};
    Arc right6{UISubsystem::Color::BLACK, (uint16_t) (0.49940968122786306*SIZE+POSITION_X), (uint16_t) (0.3447461629279811*SIZE+POSITION_Y), (uint16_t) (0.6115702479338843*SIZE), (uint16_t) (0.5218417945690673*SIZE), 177, 100, (uint16_t) (0.2361275088547816*SIZE)};

    // white border, covers the sloppy left, mid, and right
    Arc border0{UISubsystem::Color::WHITE, (uint16_t) (0.3695395513577332*SIZE+POSITION_X), (uint16_t) (0.17473435655253838*SIZE+POSITION_Y), (uint16_t) (0.12750885478158205*SIZE), (uint16_t) (0.089728453364817*SIZE), 190, 101, (uint16_t) (0.023612750885478158*SIZE)};
    Arc border1{UISubsystem::Color::WHITE, (uint16_t) (0.5584415584415584*SIZE+POSITION_X), (uint16_t) (0.5430932703659976*SIZE+POSITION_Y), (uint16_t) (0.8311688311688312*SIZE), (uint16_t) (0.6422668240850059*SIZE), 38, 314, (uint16_t) (0.023612750885478158*SIZE)};
    Arc border2{UISubsystem::Color::WHITE, (uint16_t) (0.5230224321133412*SIZE+POSITION_X), (uint16_t) (0.34238488783943327*SIZE+POSITION_Y), (uint16_t) (0.602125147579693*SIZE), (uint16_t) (0.6682408500590319*SIZE), 240, 146, (uint16_t) (0.023612750885478158*SIZE)};
    Arc border3{UISubsystem::Color::WHITE, (uint16_t) (0.9055489964580874*SIZE+POSITION_X), (uint16_t) (0.28807556080283353*SIZE+POSITION_Y), (uint16_t) (0.8240850059031877*SIZE), (uint16_t) (1.050767414403778*SIZE), 13, 290, (uint16_t) (0.023612750885478158*SIZE)};
    Arc border4{UISubsystem::Color::WHITE, (uint16_t) (0.5737898465171193*SIZE+POSITION_X), (uint16_t) (0.16056670602125148*SIZE+POSITION_Y), (uint16_t) (0.4203069657615112*SIZE), (uint16_t) (0.717827626918536*SIZE), 320, 280, (uint16_t) (0.023612750885478158*SIZE)};
    Arc border5{UISubsystem::Color::WHITE, (uint16_t) (0.30932703659976385*SIZE+POSITION_X), (uint16_t) (0.6564344746162928*SIZE+POSITION_Y), (uint16_t) (0.448642266824085*SIZE), (uint16_t) (0.5100354191263282*SIZE), 173, 140, (uint16_t) (0.023612750885478158*SIZE)};
    Arc border6{UISubsystem::Color::WHITE, (uint16_t) (0.667060212514758*SIZE+POSITION_X), (uint16_t) (0.3789846517119244*SIZE+POSITION_Y), (uint16_t) (0.448642266824085*SIZE), (uint16_t) (0.5100354191263282*SIZE), 130, 100, (uint16_t) (0.023612750885478158*SIZE)};
    Arc border7{UISubsystem::Color::WHITE, (uint16_t) (1.0956316410861866*SIZE+POSITION_X), (uint16_t) (0.3530106257378985*SIZE+POSITION_Y), (uint16_t) (0.42502951593860683*SIZE), (uint16_t) (1.050767414403778*SIZE), 327, 290, (uint16_t) (0.023612750885478158*SIZE)};
    Arc border8{UISubsystem::Color::WHITE, (uint16_t) (0.1192443919716647*SIZE+POSITION_X), (uint16_t) (0.6434474616292798*SIZE+POSITION_Y), (uint16_t) (0.49586776859504134*SIZE), (uint16_t) (0.33293978748524206*SIZE), 38, 332, (uint16_t) (0.023612750885478158*SIZE)};
    Arc border9{UISubsystem::Color::WHITE, (uint16_t) (-0.42621015348288077*SIZE+POSITION_X), (uint16_t) (0.5631641086186541*SIZE+POSITION_Y), (uint16_t) (1.0554899645808737*SIZE), (uint16_t) (0.8571428571428571*SIZE), 87, 58, (uint16_t) (0.023612750885478158*SIZE)};
    Arc border10{UISubsystem::Color::WHITE, (uint16_t) (0.26092089728453366*SIZE+POSITION_X), (uint16_t) (0.42266824085005905*SIZE+POSITION_Y), (uint16_t) (0.3282172373081464*SIZE), (uint16_t) (0.5029515938606848*SIZE), 260, 175, (uint16_t) (0.023612750885478158*SIZE)};
    Arc border11{UISubsystem::Color::WHITE, (uint16_t) (0.48760330578512395*SIZE+POSITION_X), (uint16_t) (0.3577331759149941*SIZE+POSITION_Y), (uint16_t) (0.1487603305785124*SIZE), (uint16_t) (0.20543093270365997*SIZE), 10, 316, (uint16_t) (0.023612750885478158*SIZE)};
    Line border12{UISubsystem::Color::WHITE, (uint16_t) (0.1050767414403778*SIZE+POSITION_X), (uint16_t) (0.3778040141676505*SIZE+POSITION_Y), (uint16_t) (0.09445100354191263*SIZE+POSITION_X), (uint16_t) (0.5867768595041323*SIZE+POSITION_Y), (uint16_t) (0.023612750885478158*SIZE)};
    Line border13{UISubsystem::Color::WHITE, (uint16_t) (0.27508854781582054*SIZE+POSITION_X), (uint16_t) (0.1641086186540732*SIZE+POSITION_Y), (uint16_t) (0.27508854781582054*SIZE+POSITION_X), (uint16_t) (0.7662337662337663*SIZE+POSITION_Y), (uint16_t) (0.023612750885478158*SIZE)};
    Line border14{UISubsystem::Color::WHITE, (uint16_t) (0.42621015348288077*SIZE+POSITION_X), (uint16_t) (0.1652892561983471*SIZE+POSITION_Y), (uint16_t) (0.525383707201889*SIZE+POSITION_X), (uint16_t) (0.4734356552538371*SIZE+POSITION_Y), (uint16_t) (0.023612750885478158*SIZE)};
    Line border15{UISubsystem::Color::WHITE, (uint16_t) (0.49586776859504134*SIZE+POSITION_X), (uint16_t) (0.38134592680047225*SIZE+POSITION_Y), (uint16_t) (0.5525383707201889*SIZE+POSITION_X), (uint16_t) (0.704840613931523*SIZE+POSITION_Y), (uint16_t) (0.023612750885478158*SIZE)};
    Line border16{UISubsystem::Color::WHITE, (uint16_t) (0.3624557260920897*SIZE+POSITION_X), (uint16_t) (0.1357733175914994*SIZE+POSITION_Y), (uint16_t) (0.27744982290436837*SIZE+POSITION_X), (uint16_t) (0.17355371900826447*SIZE+POSITION_Y), (uint16_t) (0.023612750885478158*SIZE)};
    Line border17{UISubsystem::Color::WHITE, (uint16_t) (0.8819362455726092*SIZE+POSITION_X), (uint16_t) (0.33412042502951594*SIZE+POSITION_Y), (uint16_t) (0.9020070838252656*SIZE+POSITION_X), (uint16_t) (0.5324675324675324*SIZE+POSITION_Y), (uint16_t) (0.023612750885478158*SIZE)};
    Arc border18{UISubsystem::Color::WHITE, (uint16_t) (0.2892561983471074*SIZE+POSITION_X), (uint16_t) (0.4474616292798111*SIZE+POSITION_Y), (uint16_t) (1.2396694214876034*SIZE), (uint16_t) (1.0059031877213696*SIZE), 140, 117, (uint16_t) (0.023612750885478158*SIZE)};
    Arc border19{UISubsystem::Color::WHITE, (uint16_t) (0.4462809917355372*SIZE+POSITION_X), (uint16_t) (0.6635182998819362*SIZE+POSITION_Y), (uint16_t) (0.19716646989374262*SIZE), (uint16_t) (0.15466351829988192*SIZE), 90, 54, (uint16_t) (0.011806375442739079*SIZE)};
    Line border20{UISubsystem::Color::WHITE, (uint16_t) (0.5619834710743802*SIZE+POSITION_X), (uint16_t) (0.6953955135773318*SIZE+POSITION_Y), (uint16_t) (0.5194805194805194*SIZE+POSITION_X), (uint16_t) (0.706021251475797*SIZE+POSITION_Y), (uint16_t) (0.011806375442739079*SIZE)};
    Line border21{UISubsystem::Color::WHITE, (uint16_t) (0.4197166469893743*SIZE+POSITION_X), (uint16_t) (0.5602125147579693*SIZE+POSITION_Y), (uint16_t) (0.3772136953955136*SIZE+POSITION_X), (uint16_t) (0.5755608028335301*SIZE+POSITION_Y), (uint16_t) (0.015348288075560802*SIZE)};
    Line border22{UISubsystem::Color::WHITE, (uint16_t) (0.4315230224321133*SIZE+POSITION_X), (uint16_t) (0.5909090909090909*SIZE+POSITION_Y), (uint16_t) (0.3772136953955136*SIZE+POSITION_X), (uint16_t) (0.5708382526564345*SIZE+POSITION_Y), (uint16_t) (0.015348288075560802*SIZE)};
    Line border23{UISubsystem::Color::WHITE, (uint16_t) (0.42621015348288077*SIZE+POSITION_X), (uint16_t) (0.5726092089728453*SIZE+POSITION_Y), (uint16_t) (0.40613931523022434*SIZE+POSITION_X), (uint16_t) (0.5832349468713105*SIZE+POSITION_Y), (uint16_t) (0.023612750885478158*SIZE)};










};