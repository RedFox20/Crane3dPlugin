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

void UCraneSimulationComponent::AddLineZ(float axisValue, float multiplier)
{
    ForceCable = axisValue * multiplier;
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
    GEngine->AddOnScreenDebugMessage(9, 5.0f, FColor::Red, FString::Printf(L"Fcabl: %.2f", ForceCable));
}

void UCraneSimulationComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // update model with parameters
    Model->Type = (crane3d::ModelType)ModelType;

    Model->Mrail = RailMass;
    Model->Mcart = CartMass;
    Model->Mpayload = PayloadMass;
    Model->G = Gravity;

    Model->RailFriction = RailFriction;
    Model->CartFriction = CartFriction;
    Model->LineFriction = LineFriction;

    Model->RailLimitMin = RailLimitMin;
    Model->RailLimitMax = RailLimitMax;
    Model->CartLimitMin = CartLimitMin;
    Model->CartLimitMax = CartLimitMax;
    Model->LineLimitMin = LineLimitMin;
    Model->LineLimitMax = LineLimitMax;

    crane3d::ModelState state = Model->Update(DeltaTime, ForceRail, ForceCart, ForceCable);
    UpdateVisibleFields(state);

    // reset all forces for this frame
    ForceRail = 0.0;
    ForceCart = 0.0;
    ForceCable = 0.0;

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

