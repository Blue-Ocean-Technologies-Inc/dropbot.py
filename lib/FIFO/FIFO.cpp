// -*- coding: utf-8 -*-
/**
 @file		FIFO.cpp
 @brief	 	1Byte Software FIFO(First-in, First-out)

 This library provides variable length ring buffering with software FIFO(First-in, First-out).
 This library overwrites when the buffer is full.

 @author	T.Kawamura
 @version	2.1
 @date		2016-11-20	T.Kawamura	Written for AVR.
 @date		2017-03-28	T.Kawamura	Fork for mbed/C++.
 @date		2017-03-29	T.Kawamura	Add template.
 @date		2017-06-17	T.Kawamura	Chanege to overwrite.

 @see
 Copyright (C) 2016-2017 T.Kawamura.
 Released under the MIT license.
 http://opensource.org/licenses/mit-license.php

*/

#include <stdint.h>
#include "FIFO.hpp"


template <class T>
FIFO<T>::FIFO(uint32_t bufsize){
	buffer = new T[bufsize];
	size = bufsize;
	getindex = 0;
	putindex = 0;
	count = 0;
	return;
}

template <class T>
FIFO<T>::~FIFO(void){
	delete[] buffer;
	return;
}

template <class T>
void FIFO<T>::clear(void){
  count = 0;
  getindex = 0;
  putindex = 0;

	return;
}

template <class T>
uint32_t FIFO<T>::getsize(void){
  return size;
}

template <class T>
T FIFO<T>::get(void){
  T getdata;

  if ( count <= 0 ){
    return 0;
  }

  getdata = buffer[getindex];
  getindex++;
  if ( getindex >= size ){	//When the index is in the terminus of the buffer
    getindex = 0;
  }
  count--;

  return getdata;
}

template <class T>
T FIFO<T>::peek(void){
  T getdata;

  if ( count <= 0 ){	//When the buffer is empty
    return 0;
  }
  getdata = buffer[getindex];
  return getdata;
}

template <class T>
void FIFO<T>::put(const T putdata){
  buffer[putindex] = putdata;
  putindex++;
  if ( putindex >= size ){	// When the index is in the terminus of the buffer
    putindex = 0;
  }
  count++;

  return;
}

template <class T>
uint32_t FIFO<T>::available(void){
  uint32_t c = 0;

  c = count;

  return c;
}
