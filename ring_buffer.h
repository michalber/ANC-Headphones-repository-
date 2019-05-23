#pragma once
#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#ifndef FRAMES_PER_BUFFER
#define FRAMES_PER_BUFFER  (2048)
#endif //FRAMES_PER_BUFFER

#include <armadillo>

#ifndef DEBUG
	#define DEBUG 0
#endif // DEBUG

namespace RingBuffer {
	template <typename T>

	class RingBuffer {

		volatile int head;
		volatile int tail;
		volatile int numOfElem;
		T* buff;
		volatile bool fullFlag;
		arma::vec temp;


	public:

		/**
			@brief Initializes the given ring buffer structure.

			@author	Micha³ Berdzik
			@version 0.0.1 10-05-2019
			
			@param dataBufferSize size in bytes of the dataBuffer
		*/
		RingBuffer();
		RingBuffer(int);
		~RingBuffer();

		/**
			@brief Sets size of the ring buffer.
			
			@author	Micha³ Berdzik
			@version 0.0.1 10-05-2019

		*/
		void RingBuffer_SetSize(int);

		/**
			@brief Clears contents of the given ring buffer.
			
			@author	Micha³ Berdzik
			@version 0.0.1 10-05-2019

			@return true if the ring buffer is cleared successfully, false otherwise
		*/
		bool RingBuffer_Clear();

		/**
			@brief Checks if the given ring buffer is empty.
			
			@author	Micha³ Berdzik
			@version 0.0.1 10-05-2019

			@return true if the ring buffer holds no data, false otherwise
		*/
		bool RingBuffer_IsEmpty();

		/**
			@brief Checks if the given ring buffer is full.
			
			@author	Micha³ Berdzik
			@version 0.0.1 10-05-2019

			@return true if the ring buffer is full, false otherwise
		*/
		bool RingBuffer_IsFull();

		/**
			@brief Gets the length (in bytes) of the data stored in the given ring buffer.
			
			@author	Micha³ Berdzik
			@version 0.0.1 10-05-2019

			@return length (in bytes) of the data stored in the ring buffer
		*/
		int RingBuffer_GetLen();

		/**
			@brief Returns the capacity (in bytes) of the given buffer.
			
			@author	Micha³ Berdzik
			@version 0.0.1 10-05-2019

			@return capacity (in bytes) of the ring buffer (how much characters can it store)
		*/
		int RingBuffer_GetCapacity();
		/**
			@brief Function to check if size of buffer is critical (to load new data)

			@author	Micha³ Berdzik
			@version 0.0.1 10-05-2019

			@return true if length < 20% of its capacity
		*/
		bool RingBuffer_GetCapacityAlarm();

		/**
			@brief Appends a single data type T to the ring buffer. The stored data length will be increased by 1.
			
			@author	Micha³ Berdzik
			@version 0.0.1 10-05-2019

			@return true if the character was added successfully, false otherwise
		*/
		bool RingBuffer_Put(T t);

		/**
			@brief Pulls out a single data type T from the ring buffer. The stored data length will be decreased by 1.
			
			@author	Micha³ Berdzik
			@version 0.0.1 10-05-2019

			@return true if the character was pulled out successfully, false otherwise
		*/
		T RingBuffer_Get();

		/**
			@brief Pulls out a current position of RingBuffer's head
			
			@author	Micha³ Berdzik
			@version 0.0.1 10-05-2019

			@return current RingBuffer's head position
		*/
		int RingBuffer_GetHead();

		/**
			@brief Pulls out a current position of RingBuffer's tail
			
			@author	Micha³ Berdzik
			@version 0.0.1 10-05-2019

			@return current RingBuffer's tail position
		*/
		int RingBuffer_GetTail();


		arma::vec RingBuffer_GetBufferAsVec();

		void RingBuffer_PutVecIntoBuffer(arma::vec);
	};


	// --------------------------------------------------------------------------------------
	// ----------------------------- FUNCTIONS DEFINITIONS  ---------------------------------
	// --------------------------------------------------------------------------------------

#include <assert.h>

	template<typename T>
	RingBuffer<T>::RingBuffer()
	{
	}
	// -------------------------------------------------------------------------------------------------------------
	template<typename T>
	RingBuffer<T>::RingBuffer(int dataBufferSize)
	{
		assert(dataBufferSize > 0);

		if ((dataBufferSize > 0)) {
			head = 0;
			tail = 0;
			fullFlag = false;
			numOfElem = dataBufferSize;
			buff = (T*)calloc(numOfElem, sizeof(T));
			temp.set_size(FRAMES_PER_BUFFER);
			temp.fill(0);
		}
	}
	// -------------------------------------------------------------------------------------------------------------

	template<typename T>
	bool RingBuffer<T>::RingBuffer_Clear()
	{
		head = 0;
		tail = 0;
		fullFlag = false;
		return true;
	}
	// -------------------------------------------------------------------------------------------------------------

	template<typename T>
	bool RingBuffer<T>::RingBuffer_IsEmpty()
	{
		if (this->RingBuffer_GetLen() != 0) {
			return false;
		}

		return true;
	}
	template<typename T>
	bool RingBuffer<T>::RingBuffer_IsFull()
	{
		return fullFlag;
	}
	// -------------------------------------------------------------------------------------------------------------
	template<typename T>
	int RingBuffer<T>::RingBuffer_GetLen()
	{

		if (fullFlag == true)
			return numOfElem;
		else if (tail <= head) {
			return head - tail;
		}
		else {
			return numOfElem - tail + head;
		}
	}
	// -------------------------------------------------------------------------------------------------------------
	template<typename T>
	int RingBuffer<T>::RingBuffer_GetCapacity()
	{
		return numOfElem;
	}
	// -------------------------------------------------------------------------------------------------------------
	template<typename T>
	bool RingBuffer<T>::RingBuffer_GetCapacityAlarm()
	{
		if (this->RingBuffer_GetLen() <= (int)(1 / 10 * numOfElem))
			return 1;
		else return 0;
	}
	// -------------------------------------------------------------------------------------------------------------

	template<typename T>
	bool RingBuffer<T>::RingBuffer_Put(T t)
	{
		if (this->RingBuffer_GetLen() < numOfElem) {
			*(buff + head) = t;			
			temp(head) = t;
			if (head == numOfElem - 1)
				head = 0;
			else
				head++;
			if (head == tail)
				fullFlag = true;
			return true;
		}
		else {
#if DEBUG
			if (fullFlag) {
				cout << endl;
				cout << "-------------------------------------------------------------------------------------------" << endl;
				cout << "DEBUG MESSAGE:" << endl;
				cout << "RingBuffer is full!!" << endl;
				cout << "-------------------------------------------------------------------------------------------" << endl;
				cout << endl; \
			}
#endif
			return false;
		}
	}
	// -------------------------------------------------------------------------------------------------------------

	template<typename T>
	T RingBuffer<T>::RingBuffer_Get()
	{
		T c;

		if (this->RingBuffer_IsEmpty() != true) {
			c = *(buff + tail);
			*(buff + tail) = 0;
			if (tail == numOfElem - 1)
				tail = 0;
			else
				tail++;
			if (fullFlag == true)
				fullFlag = false;
			return c;
		}
	}
	// -------------------------------------------------------------------------------------------------------------
	template<typename T>
	int RingBuffer<T>::RingBuffer_GetHead()
	{
		return head;
	}
	// -------------------------------------------------------------------------------------------------------------
	template<typename T>
	int RingBuffer<T>::RingBuffer_GetTail()
	{
		return tail;
	}
	// -------------------------------------------------------------------------------------------------------------
	template<typename T>
	arma::vec RingBuffer<T>::RingBuffer_GetBufferAsVec()
	{
		/*for (int i = 0; i < FRAMES_PER_BUFFER; i++) {
			temp(i) = this->RingBuffer_Get();
		}*/
		head = 0;
		tail = 0;
		fullFlag = false;

		return temp;
	}
	// -------------------------------------------------------------------------------------------------------------
	template<typename T>
	void RingBuffer<T>::RingBuffer_PutVecIntoBuffer(arma::vec in)
	{
		for (int i = 0; i < in.size(); i++) {
			this->RingBuffer_Put(in(i));
		}
	}
	// -------------------------------------------------------------------------------------------------------------
	template<typename T>
	RingBuffer<T>::~RingBuffer()
	{
	}
	// -------------------------------------------------------------------------------------------------------------
	template<typename T>
	void RingBuffer<T>::RingBuffer_SetSize(int size)
	{
		assert(size > 0);
		numOfElem = size;
		free(buff);
		buff = (T*)calloc(numOfElem, sizeof(T));
		temp.set_size(FRAMES_PER_BUFFER);
		temp.fill(0);
	}
	// -------------------------------------------------------------------------------------------------------------
}

#endif // !_RING_BUFFER_H_