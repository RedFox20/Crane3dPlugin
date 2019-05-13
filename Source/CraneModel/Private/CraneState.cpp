// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#include "CraneState.h"
#include <cstdio>

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    void CraneState::Print() const
    {
        printf("Rail: %.2f Cart: %.2f Line: %.2f X: %.2f Y: %.2f Z: %.2f\n",
                RailOffset, CartOffset, LiftLine, PayloadX, PayloadY, PayloadZ);
    }

    //////////////////////////////////////////////////////////////////////
}
