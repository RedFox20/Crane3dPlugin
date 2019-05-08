// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
using UnrealBuildTool;
public class CraneModel : ModuleRules
{
    public CraneModel(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
        bFasterWithoutUnity = true;
        bEnforceIWYU = true;
        bPrecompile = false;
        PublicDependencyModuleNames.AddRange(
            new [] { "Core", "CoreUObject", "Engine" });
    }
}
