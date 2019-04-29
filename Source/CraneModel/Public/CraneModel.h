// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
#pragma once
#include "Modules/ModuleManager.h"

class FCraneModelModule : public IModuleInterface
{
public:

    /** IModuleInterface implementation */
    void StartupModule() override;
    void ShutdownModule() override;
};
