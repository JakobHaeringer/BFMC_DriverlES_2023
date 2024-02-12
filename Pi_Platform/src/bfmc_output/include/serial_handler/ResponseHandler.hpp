/**
 * @file ResponseHandler.hpp
 * @author Bosch Engineering GmbH
 * @brief The header file is for the response handling of messages received by the Raspberry Pi from the STM board.
 * @version 1.0
 * @date 2023-07-17
 *
 * @copyright  Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers, All rights reserved.
 */
#ifndef MESSAGE_HANDLER_HPP
#define MESSAGE_HANDLER_HPP

#include "serial_handler/Message.hpp"
#include <boost/function.hpp>
#include <deque>
#include <iostream>
#include <map>
#include <vector>

/** @brief This class builds the base for handling the received messages from the STM board via serial.
 *
 * Upon sending a message the Raspberry Pi expects a confirmation from the STM board for each command sent, so a stable communication can be
 * guaranteed. The response is a string with a specified format. The ResponseHandler decodes and interprets that string.
 */
class BaseResponseHandler {
public:
    virtual void operator()(const char*, const size_t) = 0;
    virtual void _run()                                = 0;
    /** @brief Deactivate the connection to the STM board. */
    virtual void deactive();

protected:
    /** @brief Remain true, while the _run executing. */
    bool m_active;
};

/** @brief This class handles the receiving of the messages from the STM board via serial. */
class ResponseHandler : public BaseResponseHandler {
public:
    typedef boost::function<void(std::string)>  CallbackFncType;
    typedef boost::function<void(std::string)>* CallbackFncPtrType;
    typedef std::vector<CallbackFncPtrType>     CallbackFncContainer;

    template <class T>
    static CallbackFncPtrType createCallbackFncPtr(void (T::*f)(std::string), T* obj) {
        return new CallbackFncType(std::bind1st(std::mem_fun(f), obj));
    }

    /**
     * @brief Create a callback function object that can be used to call a function to handle certain message responses.
     *
     * @param f A void pointer to the name of the function that should be registered as a callback function.
     * @return New static callback function pointer object.
     */
    static CallbackFncPtrType createCallbackFncPtr(void (*f)(std::string));

    /** @brief Construct a new Response Handler object. */
    ResponseHandler();
    /**
     * @brief Copy the characters from input buffer to the message buffer.
     *
     * @param buffer Pointer to the buffer.
     * @param bytes_transferred Number bytes transferred.
     */
    void operator()(const char*, const size_t);

    /** @brief while the m_actice is true, the _run executing cyclically, read a character from the message buffer and sends it to the processor.*/
    void _run();

    /**
     * @brief   Attach the callback function  to the response key word. This callback function will be called automatically, when will be received the key word
     *          feedback from the STM board. More functions can be attach to the one key word.
     * @param f_action The action name that should be registered in a callback function, so the callback functions gets executed when this action is received as a response from the STM board.
     * @param waiter The callback function with which the action is associated.
     * @return new object
     */
    void attach(std::string, CallbackFncPtrType);

    /**
     * @brief After applying detach  on a certain function and message. The callback function will not be called anymore after applying this.
     * @param f_action The action name that should be removed from a callback function, so the callback functions won't get executed when this action is received as a response from the STM board.
     * @param waiter The callback function with which the action shouldn't be associated anymore.
     * @return new object
     */
    void detach(std::string, CallbackFncPtrType);

private:
    /**
     * @brief   Each received character is sent to this function. If the char is '@', it signales the begining of the response.
     *          If the character is new line ('/r'), it signales the end of the response.
     *          If there is any other character, it appends it to the valid response attribute.
     *
     * @param f_received_chr The character that has been received and shall be checked.
     */
    void processChr(const char);
    /**
     * @brief Check if the response was valid or not.
     */
    void checkResponse();

    /** @brief Buffered read data. */
    std::deque<char> read_msgs_;
    /** @brief Is true, when receiving a response valid from the device. */
    bool m_isResponse;
    /** @brief Buffer the valid response. */
    std::deque<char> m_valid_response;
    /** @brief Map of actions associated with a callback function. */
    std::map<std::string, CallbackFncContainer> m_keyCallbackFncMap;
};

#endif
