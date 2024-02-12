/**
 * @file ResponseHandler.cpp
 * @author Bosch Engineering GmbH
 * @brief This file implements the response handling of messages received by the Raspberry Pi from the STM board.
 * @version 1.0
 * @date 2023-07-17
 *
 * @copyright  Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers, All rights reserved.
 */
#include "serial_handler/ResponseHandler.hpp"

void BaseResponseHandler::deactive() {
    m_active = false;
}

ResponseHandler::ResponseHandler() :
    m_isResponse(false) {
    m_active = true;
}

void ResponseHandler::operator()(const char* buffer, const size_t bytes_transferred) {
    for (unsigned int i = 0; i < bytes_transferred; ++i) {
        read_msgs_.push_back(buffer[i]);
    }
}

void ResponseHandler::_run() {
    while (m_active) {
        if (!read_msgs_.empty()) {
            char l_c = read_msgs_.front();
            read_msgs_.pop_front();
            processChr(l_c);
        }
    }
}

ResponseHandler::CallbackFncPtrType ResponseHandler::createCallbackFncPtr(void (*f)(std::string)) {
    return new CallbackFncType(f);
}

void ResponseHandler::attach(std::string f_action, CallbackFncPtrType waiter) {
    if (m_keyCallbackFncMap.count(f_action) > 0) {
        CallbackFncContainer* l_container = &(m_keyCallbackFncMap[f_action]);
        l_container->push_back(waiter);
    }
    else {
        CallbackFncContainer l_container;
        l_container.push_back(waiter);
        m_keyCallbackFncMap[f_action] = l_container;
    }
}

void ResponseHandler::detach(std::string f_action, CallbackFncPtrType waiter) {
    if (m_keyCallbackFncMap.count(f_action) > 0) {
        CallbackFncContainer*          l_container = (&m_keyCallbackFncMap[f_action]);
        CallbackFncContainer::iterator it          = std::find(l_container->begin(), l_container->end(), waiter);
        if (it != l_container->end()) {
            l_container->erase(it);
        }
        else {
            std::cout << "Not found!" << std::endl;
        }
    }
    else {
        std::cout << "Container is empty!" << std::endl;
    }
}

void ResponseHandler::processChr(const char f_received_chr) {
    if (f_received_chr == '@') {
        m_isResponse = true;
    }
    else if (f_received_chr == '\r') {
        if (!m_valid_response.empty()) {
            checkResponse();
            m_valid_response.clear();
        }
        m_isResponse = false;
    }
    if (m_isResponse) {
        m_valid_response.push_back(f_received_chr);
    }
}

void ResponseHandler::checkResponse() {
    std::string l_responseFull(m_valid_response.begin(), m_valid_response.end());
    std::string l_keyStr     = l_responseFull.substr(1, 1);
    std::string l_reponseStr = l_responseFull.substr(0, l_responseFull.length());
    if (std::stoi(l_keyStr) > 0) {
        CallbackFncContainer l_cointaner = m_keyCallbackFncMap[l_keyStr];
        for (CallbackFncContainer::iterator it = l_cointaner.begin(); it != l_cointaner.end(); ++it) {
            (**it)(l_reponseStr);
        }
    }
    else {
        std::cout << "";
    }
}
