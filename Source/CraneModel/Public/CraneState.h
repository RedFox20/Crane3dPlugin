// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#pragma once

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    /**
     * Output state of the model
     */
    struct CraneState
    {
        // X distance of the rail with the cart from
        // the center of the construction frame
        double RailOffset = 0.0;
        // Y distance of the cart from the center of the rail
        double CartOffset = 0.0;
        // R lift-line length
        double LiftLine = 0.0;

        // Payload 3D coordinates
        double PayloadX = 0.0;
        double PayloadY = 0.0;
        double PayloadZ = 0.0;

        void Print() const;
    };

    //////////////////////////////////////////////////////////////////////
}
