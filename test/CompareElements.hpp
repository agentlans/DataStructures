#ifndef _COMPAREELEMENTS
#define _COMPAREELEMENTS

#include <unordered_set>
#include <algorithm>

template <class T1, class T2>
bool same(const T1& x, const T2& y) {
	return std::equal(x.begin(), x.end(), y.begin());
}

template <class T1, class T2>
bool same_elements(const T1& x, const T2& y) {
	auto x2 = x;
	auto y2 = y;
	std::sort(x2.begin(), x2.end());
	std::sort(y2.begin(), y2.end());
	return std::equal(x2.begin(), x2.end(), y2.begin());
}

#endif
