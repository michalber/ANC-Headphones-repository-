#pragma once
#ifndef _THREAD_WRAPPER_H_
#define _THREAD_WRAPPER_H_


#include <thread>
#include <mutex>
#include <vector>
#include <assert.h>
#include <chrono>

namespace Wrapper {
	/*
	 * A class that has thread object as member variable
	 */
	class ThreadWrapper
	{
		// std::thread object
		std::thread  threadHandler;

	public:
		ThreadWrapper();

		//Delete the copy constructor
		ThreadWrapper(const ThreadWrapper&) = delete;

		//Delete the Assignment opeartor
		ThreadWrapper& operator=(const ThreadWrapper&) = delete;

		// Parameterized Constructor
		ThreadWrapper(std::function<void()> func);

		// Move Constructor
		ThreadWrapper(ThreadWrapper && obj);

		//Move Assignment Operator
		ThreadWrapper & operator=(ThreadWrapper && obj);

		//Destructor
		~ThreadWrapper();

		void detach();
	};
}

#endif // !_THREAD_WRAPPER_H_