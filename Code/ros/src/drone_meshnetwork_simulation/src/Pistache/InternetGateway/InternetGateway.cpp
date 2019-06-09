/**
 * @file InternetGateway.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief sourcefile for InternetGateway
 * @version 1.0
 * @date 2019-05-28
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "InternetGateway.hpp"
#include <iostream>

using namespace Pistache;


namespace Generic {

void handleReady(const Rest::Request&, Http::ResponseWriter response) {
    response.send(Http::Code::Ok, "1");
    
}
}

InternetGateway::InternetGateway(/* args */)
:desc("Gateway API", "0.1"), httpEndpoint(nullptr),meshnetworkGateway(nullptr)
{

}

InternetGateway::~InternetGateway()
{

}

void InternetGateway::init(Communication::Internet::IGatewayCommands *IGC,uint8_t _threads)
{
    meshnetworkGateway = IGC;
    Address addr("*:9080");
    httpEndpoint = std::make_shared<Http::Endpoint>(addr);
    auto opts = Http::Endpoint::options()
            .threads(_threads)
            .flags(Tcp::Options::InstallSignalHandler);
        httpEndpoint->init(opts);
        createDescription();
        std::cout<<addr.host()<<":"<<addr.port()<<std::endl;
}

void InternetGateway::createDescription()
{
        using namespace Rest;

        Routes::Post(router, "/DroneMovement/:id/:latitude/:longitude/:height", Routes::bind(&InternetGateway::GetValueBack, this));
        Routes::Get(router, "/ready", Routes::bind(&Generic::handleReady));
}



void InternetGateway::connect()
{
    this->InternetConnectionThread = std::thread(std::bind(&InternetGateway::InternetService, this));
    
}

void InternetGateway::disconnect()
{
    httpEndpoint->shutdown();
}

void InternetGateway::GetValueBack(const Rest::Request& request, Http::ResponseWriter response)
{

    int id = request.param(":id").as<int>();
    float latitude = request.param(":latitude").as<float>();
    float longitude = request.param(":longitude").as<float>();
    int height = request.param(":height").as<int>();
    meshnetworkGateway->SendGoalRequestToDrone(id,latitude,longitude,height);
    response.send(Http::Code::Ok, std::to_string(id)+ "  " + std::to_string(latitude)+ "  " +std::to_string(longitude)+ "  " +std::to_string(height));
}

void InternetGateway::InternetService()
{
    router.initFromDescription(desc);
    httpEndpoint->setHandler(router.handler());
    httpEndpoint->serve();
    httpEndpoint->shutdown();
    InternetConnectionThread.join();
}