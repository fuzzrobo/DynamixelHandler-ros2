#ifndef MAKE_ITERATOR_CONVENIENT_HPP
#define MAKE_ITERATOR_CONVENIENT_HPP

// enum でインクリメントをするため
template<typename T> T& operator ++ (T& v     ) { v = static_cast<T>(v + 1); return v;}
template<typename T> T  operator ++ (T& v, int) { T p=v; ++v; return p;}
// vectorやsetに値が含まれているかどうかを調べる関数
template <typename T> bool is_in(const T& val, const vector<T>& v) { return std::find(v.begin(), v.end(), val) != v.end(); }
template <typename T> bool is_in(const T& val, const    set<T>& s) { return s.find(val) != s.end(); }

#endif /* MAKE_ITERATOR_CONVENIENT_HPP */