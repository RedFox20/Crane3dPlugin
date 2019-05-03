// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
#include "CraneSimulationComponent.h"
#include "EngineMinimal.h"
#include "DrawDebugHelpers.h"

UCraneSimulationComponent::UCraneSimulationComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UCraneSimulationComponent::AddRailX(float axisValue, float multiplier)
{
    ForceRail = axisValue * multiplier;
}

void UCraneSimulationComponent::AddCartY(float axisValue, float multiplier)
{
    ForceCart = axisValue * multiplier;
}

void UCraneSimulationComponent::AddWindingZ(float axisValue, float multiplier)
{
    ForceWinding = axisValue * multiplier;
}

void UCraneSimulationComponent::NextModelType()
{
    int next = (int)ModelType + 1;
    if (next > (int)ECraneModelType::NonLinearOriginal) next = 0;
    ModelType = (ECraneModelType)next;
}

void UCraneSimulationComponent::BeginPlay()
{
    Super::BeginPlay();

    Model = std::make_unique<crane3d::Model>();
    UpdateVisibleFields(Model->GetState());
}

void UCraneSimulationComponent::UpdateVisibleFields(const crane3d::ModelState& state)
{
    RailOffset = state.RailOffset;
    CartOffset = state.CartOffset;
    PayloadPosition = FVector{ (float)state.PayloadX, (float)state.PayloadY, (float)state.PayloadZ };

    GEngine->AddOnScreenDebugMessage(1, 5.0f, FColor::Red, FString::Printf(L"RailOffset: %.2f", RailOffset));
    GEngine->AddOnScreenDebugMessage(2, 5.0f, FColor::Red, FString::Printf(L"CartOffset: %.2f", CartOffset));
    GEngine->AddOnScreenDebugMessage(3, 5.0f, FColor::Red, FString::Printf(L"PayloadPos: x:%.2f y:%.2f z:%2.f", PayloadPosition.X, PayloadPosition.Y, PayloadPosition.Z));
    GEngine->AddOnScreenDebugMessage(4, 5.0f, FColor::Red, FString::Printf(L"Alfa: %.2f", state.Alfa));
    GEngine->AddOnScreenDebugMessage(5, 5.0f, FColor::Red, FString::Printf(L"Beta: %.2f", state.Beta));
    GEngine->AddOnScreenDebugMessage(6, 5.0f, FColor::Red, FString::Printf(L"Line: %.2f", state.LiftLine));

    GEngine->AddOnScreenDebugMessage(7, 5.0f, FColor::Red, FString::Printf(L"Fcart: %.2f", ForceCart));
    GEngine->AddOnScreenDebugMessage(8, 5.0f, FColor::Red, FString::Printf(L"Frail: %.2f", ForceRail));
    GEngine->AddOnScreenDebugMessage(9, 5.0f, FColor::Red, FString::Printf(L"Fwind: %.2f", ForceWinding));

    const UEnum* modelTypes = FindObject<UEnum>(ANY_PACKAGE, TEXT("ECraneModelType"));
    GEngine->AddOnScreenDebugMessage(10, 5.0f, FColor::Red, modelTypes->GetNameStringByIndex(static_cast<int>(ModelType)) );
}

void UCraneSimulationComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // update model with parameters
    Model->Type = static_cast<crane3d::ModelType>(ModelType);

    Model->Mrail = crane3d::Mass{RailMass};
    Model->Mcart = crane3d::Mass{CartMass};
    Model->Mpayload = crane3d::Mass{PayloadMass};
    Model->G = Gravity;
    Model->g = crane3d::Accel{Gravity};

    Model->RailFriction = RailFriction;
    Model->CartFriction = CartFriction;
    Model->WindingFriction = WindingFriction;

    Model->RailLimitMin = RailLimitMin;
    Model->RailLimitMax = RailLimitMax;
    Model->CartLimitMin = CartLimitMin;
    Model->CartLimitMax = CartLimitMax;
    Model->LineLimitMin = LineLimitMin;
    Model->LineLimitMax = LineLimitMax;

    using crane3d::Force;
    crane3d::ModelState state = Model->Update(DeltaTime, Force{ForceRail}, Force{ForceCart}, Force{ForceWinding});
    UpdateVisibleFields(state);

    // reset all forces for this frame
    ForceRail = 0.0;
    ForceCart = 0.0;
    ForceWinding = 0.0;

    if (RailComponent)
        RailComponent->SetRelativeLocation(FVector{ RailOffset, 0, 0 });

    if (CartComponent)
        CartComponent->SetRelativeLocation(FVector{ RailOffset, CartOffset, 0 });

    if (PayloadComponent)
        PayloadComponent->SetRelativeLocation(PayloadPosition);

    // DEBUG visualization
    if (RailComponent && CartComponent && PayloadComponent)
    {
        FVector center = RailComponent->GetOwner()->GetActorLocation() + FVector{ 0, 0, 50 };
        FVector cart = center + FVector{ RailOffset, CartOffset, CartComponent->GetComponentLocation().Z };
        FVector payload = PayloadComponent->GetComponentLocation();

        UWorld* world = GetWorld();
        DrawDebugLine(world, cart, payload, FColor(0, 0, 255), false, -1, 0, 2);
    }
}

