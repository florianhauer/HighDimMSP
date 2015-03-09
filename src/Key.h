/*
 * Key.h
 *
 *  Created on: Feb 12, 2015
 *      Author: florian
 */

#ifndef KEY_H_
#define KEY_H_

#include "Params.h"
#include <array>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <initializer_list>

template<unsigned int DIM>
class Key : public std::array<int, DIM> {
public:
	Key(){}
	Key(const Key& s){std::copy(s.begin(),s.end(),this->begin());}
	Key operator + ( const Key& b) const{
		Key c;
		std::transform(this->begin(), this->end(), b.begin(), c.begin(),
				[](float a, float b){return a + b;});
		return c;
	}
};

#endif /* KEY_H_ */
