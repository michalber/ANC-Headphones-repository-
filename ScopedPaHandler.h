#pragma once
#ifndef _SCOPED_PA_HANDLER_H_
#define _SCOPED_PA_HANDLER_H_


#include "portaudio.h"

namespace Handler {
	class ScopedPaHandler
	{
	public:
		ScopedPaHandler()
			: _result(Pa_Initialize())
		{
		}
		~ScopedPaHandler()
		{
			if (_result == paNoError)
			{
				Pa_Terminate();
			}
		}

		PaError result() const { return _result; }

	private:
		PaError _result;
	};
}

#endif // !_SCOPED_PA_HANDLER_H_