// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/SceneComponent.h"
#include "Model.h"
#include <memory>
#include "CraneSimulationComponent.generated.h"


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
    float ForceRail = 0.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Force Inputs")
    float ForceCart = 0.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crane Force Inputs")
    float ForceCable = 0.0;

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
