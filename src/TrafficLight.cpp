#include <iostream>
#include <random>
#include <future>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */


template<typename T>
T MessageQueue<T>::receive() {
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 

    std::unique_lock <std::mutex> unique_lock(_mutex);
    _condition_variable.wait(unique_lock, [this] {
        return !_queue.empty();
    });

    T latest_message = std::move(_queue.back());
    _queue.pop_back();

    return latest_message;
}

template<typename T>
void MessageQueue<T>::send(T &&msg) {
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.

    std::lock_guard <std::mutex> lock(_mutex);
    _queue.push_back(std::move(msg));
    _condition_variable.notify_one();
}


/* Implementation of class "TrafficLight" */


TrafficLight::TrafficLight() : _currentPhase(TrafficLightPhase::red),
                               _messageQueue(std::make_shared<MessageQueue<TrafficLightPhase>>()) {}

void TrafficLight::waitForGreen() {
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop 
    // runs and repeatedly calls the receive function on the message queue. 
    // Once it receives TrafficLightPhase::green, the method returns.

    while (true) {
        TrafficLightPhase current_traffic_light_phase = _messageQueue.get()->receive();

        if (current_traffic_light_phase == TrafficLightPhase::green) {
            return;
        }
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase() {
    return _currentPhase;
}

void TrafficLight::simulate() {
    // FP.2b : Finally, the private method „cycleThroughPhases“ should be started in a thread when the public method „simulate“ is called. To do this, use the thread queue in the base class. 
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

long TrafficLight::getRandomCycleDuration() {
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<> distr(4000, 6000);
    return distr(eng);
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases() {
    // FP.2a : Implement the function with an infinite loop that measures the time between two loop cycles 
    // and toggles the current phase of the traffic light between red and green and sends an update method 
    // to the message queue using move semantics. The cycle duration should be a random value between 4 and 6 seconds. 
    // Also, the while-loop should use std::this_thread::sleep_for to wait 1ms between two cycles.

    std::chrono::time_point<std::chrono::system_clock> lastUpdate = std::chrono::system_clock::now();
    long cycleDuration = getRandomCycleDuration();

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - lastUpdate).count();
        bool cycleIsOver = timeSinceLastUpdate >= cycleDuration;

        if (cycleIsOver) {
            std::unique_lock<std::mutex> lock(_mtx);

            std::cout << "Traffic light #" << _id << " switched from " << (_currentPhase == 1 ? "green" : "red");
            _currentPhase =_currentPhase == TrafficLightPhase::green
                                                ? TrafficLightPhase::red
                                                : TrafficLightPhase::green;

            std::cout << " to " << (_currentPhase == 1 ? "green" : "red") << std::endl;

            lock.unlock();


            auto send_future = std::async(std::launch::async, // force async
                                                       &MessageQueue<TrafficLightPhase>::send,
                                                       _messageQueue,
                                                       std::move(_currentPhase));
            send_future.wait();

            cycleDuration = getRandomCycleDuration();
            lastUpdate = std::chrono::system_clock::now();
        }

    }
}

