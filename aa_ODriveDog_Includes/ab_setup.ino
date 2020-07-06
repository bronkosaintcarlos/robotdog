
// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}
// C++ program to demonstrate constructors


// sign of function
int sgn(float val){
  if(val > 0) return 1;
  else if (val < 0) return -1;
  else return 0;
}

