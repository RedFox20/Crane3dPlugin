// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/SceneComponent.h"
#include "Model.h"
#include <memory>
#include "CraneSimulationComponent.generated.h"

UENUM(BlueprintType)
enum class ECraneModelType : uint8
{
    // The most basic and foolproof crane model
    Linear,
        
    // Variation of the first linear model
    Linear2,

    // Non-linear model with constant pendulum length with 2 control forces.
    // LiftLine (Fline) is ignored
    NonLinearConstantLine,

    // Non-linear fully dynamic model with all 3 forces
    NonLinearComplete,

    // Original non-linear fully dynamic model with all 3 forces and refined friction formulae
    NonLinearOriginal,
};


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class CRANEMODEL_API UCraneSimulationComponent : public UActorComponent
{
    GENERATED_BODY()

    std::unique_ptr<crane3d::Model> Model; // need a stable pointer for UE4 editor.

public:	

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Components")
    USceneComponent* RailComponent = nullptr;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Components")
    USceneComponent* CartComponent = nullptr;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Components")
    USceneComponent* PayloadComponent = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Force Inputs")
    float ForceRail = 0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Force Inputs")
    float ForceCart = 0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Force Inputs")
    float ForceWinding = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Parameters")
    ECraneModelType ModelType = ECraneModelType::Linear;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Parameters")
    float RailMass = 2.2f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Parameters")
    float CartMass = 1.155f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Parameters")
    float PayloadMass = 1.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Parameters")
    float Gravity = 9.81f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Parameters")
    float RailFriction = 100.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Parameters")
    float CartFriction = 82.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Parameters")
    float WindingFriction = 75.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Limits")
    float RailLimitMin = -30;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Limits")
    float RailLimitMax = +30;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Limits")
    float CartLimitMin = -35;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Limits")
    float CartLimitMax = +35;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Limits")
    float LineLimitMin = 5;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Limits")
    float LineLimitMax = 90;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Crane Outputs")
    float RailOffset;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Crane Outputs")
    float CartOffset;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Crane Outputs")
    FVector PayloadPosition;

    UCraneSimulationComponent();
    
    // Adds force to the rail across the X axis
    UFUNCTION(BlueprintCallable, Category = "Crane Force Inputs")
    void AddRailX(float axisValue, float multiplier);
    
    // Adds force to the cart across the Y axis
    UFUNCTION(BlueprintCallable, Category = "Crane Force Inputs")
    void AddCartY(float axisValue, float multiplier);
    
    // Adds force to the winding mechanism across the Z axis
    UFUNCTION(BlueprintCallable, Category = "Crane Force Inputs")
    void AddWindingZ(float axisValue, float multiplier);

    // Switches to the next simulation model
    UFUNCTION(BlueprintCallable, Category = "Crane Misc. Inputs")
    void NextModelType();

protected:
    void BeginPlay() override;
    void UpdateVisibleFields(const crane3d::ModelState& state);

public:	
    void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
};
