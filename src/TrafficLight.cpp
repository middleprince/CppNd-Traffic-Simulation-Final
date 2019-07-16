#include <iostream>
#include <random>
#include "TrafficLight.h"
#include <thread>
#include <future>

/* Implementation of class "MessageQueue" */

 
template <typename T>
T MessageQueue<T>::receive()
{
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 
    std::unique_lock<std::mutex> mtx(_mutexMSG);
    _condMSG.wait(mtx, [this]() {return !_queue.empty();} );
    T msg = _queue.back();
    _queue.pop_back();
    
    return msg;
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.
    std::lock_guard<std::mutex> mtx(_mutexMSG);
    _queue.push_back(std::move(msg)); 
    _condMSG.notify_one();
} 


/* Implementation of class "TrafficLight" */

 
TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
    queueLight = std::make_shared<MessageQueue<TrafficLightPhase> >();
}

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop 
    // runs and repeatedly calls the receive function on the message queue. 
    // Once it receives TrafficLightPhase::green, the method returns.
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        //auto msg = getCurrentPhase();
        auto msg =  queueLight->receive(); 
        setCurrentPhase(msg);

        if (msg == TrafficLightPhase::green)
            return;
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    // FP.2b : Finally, the private method „cycleThroughPhases“ should be started 
    // in a thread when the public method „simulate“ is called. To do this, use 
    // the thread queue in the base class. 
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));

}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    // FP.2a : Implement the function with an infinite loop that measures the time between two loop cycles 
    // and toggles the current phase of the traffic light between red and green and sends an update method 
    // to the message queue using move semantics. The cycle duration should be a random value between 4 
    // and 6 seconds. 
    // Also, the while-loop should use std::this_thread::sleep_for to wait 1ms between two cycles. 
    //
    std::unique_lock<std::mutex> mtx(_mutex);
    std::cout << "Traffic Light#" << _id << "::CycleThroughPhases: thread id=" 
              << std::this_thread::get_id() << std::endl;
    mtx.unlock();
    
    double cycleDurationLow = 4, cycleDurationHigh = 6 ; // duration of a single simulation cycle in ms
    // init stop watch
    std::chrono::time_point<std::chrono::system_clock> lastUpdate = std::chrono::system_clock::now(); 

    while (true) 
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        long timeSinceLastUpdate ;
        
        timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - lastUpdate).count();
        
        if (timeSinceLastUpdate >= cycleDurationLow && timeSinceLastUpdate <= cycleDurationHigh) {
            // send trafficPhase to messagequeue
            if (_currentPhase == TrafficLightPhase::red) 
                _currentPhase = TrafficLightPhase::green; 
            else
                _currentPhase = TrafficLightPhase::red;

            auto msg = getCurrentPhase();
            auto is_sent = std::async(std::launch::async, &MessageQueue<TrafficLightPhase>::send, queueLight, std::move(msg));
            is_sent.wait();
            
            // reset stop watch for next cycle
            lastUpdate = std::chrono::system_clock::now();
        }
        
    }
}

