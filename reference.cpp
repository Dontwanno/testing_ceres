template<typename T> void MyClass::set(T value);

template<> void MyClass::set<int>(int value)
{
    intValue_=value;
}

template<> void MyClass::set<std::string>(std::string value)
{
    stringValue_=value;
}