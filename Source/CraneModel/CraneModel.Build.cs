// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class CraneModel : ModuleRules
{
	public CraneModel(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
        bFasterWithoutUnity = true;
        bEnforceIWYU = true;
	    bPrecompile = false;

		PublicDependencyModuleNames.AddRange(new [] { "Core", "CoreUObject", "Engine" });
	}
}
