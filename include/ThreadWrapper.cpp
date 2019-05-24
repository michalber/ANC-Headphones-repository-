#include "../ANC-Headphones-repository-/ThreadWrapper.h"


namespace Wrapper {
	ThreadWrapper::ThreadWrapper() {}

	// Parameterized Constructor
	ThreadWrapper::ThreadWrapper(std::function<void()> func) : threadHandler(func)
	{}

	// Move Constructor
	ThreadWrapper::ThreadWrapper(ThreadWrapper && obj) : threadHandler(std::move(obj.threadHandler))
	{
		//std::cout << "Move Constructor is called" << std::endl;
	}

	//Move Assignment Operator
	ThreadWrapper & ThreadWrapper::operator=(ThreadWrapper && obj)
	{
		//std::cout << "Move Assignment is called" << std::endl;
		if (threadHandler.joinable())
			threadHandler.join();
		threadHandler = std::move(obj.threadHandler);
		return *this;
	}

	// Destructor
	ThreadWrapper::~ThreadWrapper()
	{
		if (threadHandler.joinable())
			threadHandler.join();
	}
	void ThreadWrapper::detach()
	{
		threadHandler.detach();
	}
}