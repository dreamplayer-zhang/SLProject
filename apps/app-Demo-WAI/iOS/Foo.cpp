//
//  Foo.cpp
//  Demo-SL
//
//  Created by cpvrLab on 17.04.20.
//

#include "Foo.hpp"
#include <Utils.h>

Foo::Foo()
{
    Utils::log("Foo", "Fooooo!");
    _text = "here I am";
}

void Foo::hereIAm()
{
    Utils::log("Foo", _text.c_str());
}
