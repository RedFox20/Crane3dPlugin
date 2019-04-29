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
        
    // Non-linear model with constant pendulum length with 2 control forces.
    // LiftLine (Fline) is ignored
    NonLinearConstantLine,

    // Non-linear fully dynamic model with all 3 forces
    NonLinearComplete,
};


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class CRANEMODEL_API UCraneSimulationComponent : public UActorComponent
{
    GENERATED_BODY()

    // in order to allow live editing, we need
    // to store these as pointers
    std::unique_ptr<crane3d::Model> Model;

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
    float ForceCable = 0;

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
    float LineFriction = 75.0f;

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

    // Sets default values for this component's properties
    UCraneSimulationComponent();

protected:
    void BeginPlay() override;
    void UpdateVisibleFields(const crane3d::ModelState& state);

public:	
    void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
};
