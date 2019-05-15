// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
// Distributed under MIT License
#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/SceneComponent.h"
#include "Model.h"
#include <memory>
#include "CraneSimulationComponent.generated.h"

/**
 * Type of crane simulation used by CraneSimulationComponent
 */
UENUM(BlueprintType)
enum class ECraneModelType : uint8
{
    // Non-linear fully dynamic model with all 3 forces
    NonLinearComplete,

    // The most basic crane model with minimum pendulum movement
    Linear,

    // Non-linear model with constant pendulum length with 2 control forces.
    // LiftLine (Fline) is ignored
    NonLinearConstLine,

    // Original non-linear fully dynamic model with all 3 forces and refined friction formulae
    NonLinearOriginal,
};


/**
 * Adapts crane simulation and parameters as an actor component
 */
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class CRANEMODEL_API UCraneSimulationComponent : public UActorComponent
{
    GENERATED_BODY()

    std::unique_ptr<crane3d::Model> Model; // need a stable pointer for UE4 editor.

public:	

    // This should be an invisible node which marks the 0,0,0 of the crane system
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Components")
    USceneComponent* CenterComponent = nullptr;

    // crane rail (X-axis in the model)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Components")
    USceneComponent* RailComponent = nullptr;

    // crane cart (Y-axis in the model)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Components")
    USceneComponent* CartComponent = nullptr;

    // payload at PayloadPosition
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Components")
    USceneComponent* PayloadComponent = nullptr;

    // Input force driving the rail (reset every tick) - see AddRailX()
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Force Inputs")
    float ForceRail = 0;

    // Input force driving the cart (reset every tick) - see AddCartY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Force Inputs")
    float ForceCart = 0;
    
    // Input force driving the cable winding (reset every tick) - see AddWindingZ()
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Force Inputs")
    float ForceWinding = 0;

    // Crane simulation type
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Parameters")
    ECraneModelType ModelType = ECraneModelType::NonLinearComplete;

    // Mass of the rail (kg)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Parameters")
    float RailMass = 2.2f;

    // Mass of the cart (kg)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Parameters")
    float CartMass = 1.155f;

    // Mass of the payload (kg)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Parameters")
    float PayloadMass = 1.0f;

    // Gravity constant (m/s^2)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Parameters")
    float Gravity = 9.81f;

    // Min limit for the rail (cm)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Limits")
    float RailLimitMin = -28;

    // Max limit for the rail (cm)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Limits")
    float RailLimitMax = +28;
    
    // Min limit for the cart (cm)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Limits")
    float CartLimitMin = -39;
    
    // Max limit for the cart (cm)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Limits")
    float CartLimitMax = +39;
    
    // Min limit for the line (cm)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Limits")
    float LineLimitMin = 18;
    
    // Max limit for the line (cm)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Limits")
    float LineLimitMax = 70;

    // OUTPUT: X-offset of the rail and Y-offset of the cart (cm)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Crane Outputs")
    FVector CartPosition;

    // OUTPUT: Payload 3D position, offset to CenterComponent
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Crane Outputs")
    FVector PayloadPosition;

    UCraneSimulationComponent();
    
    UFUNCTION(BlueprintCallable, Category = "Crane Initializer")
    void SetCraneComponents(USceneComponent* center,
                            USceneComponent* rail,
                            USceneComponent* cart,
                            USceneComponent* payload);

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
    void UpdateVisibleFields(const crane3d::CraneState& state);
    void UpdateModelParameters();
    void UpdateVisibleComponents();

public:	
    void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
};
