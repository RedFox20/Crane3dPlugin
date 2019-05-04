// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
#include "CraneSimulationComponent.h"
#include "EngineMinimal.h"
#include "DrawDebugHelpers.h"

UCraneSimulationComponent::UCraneSimulationComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UCraneSimulationComponent::SetCraneComponents(USceneComponent* center,
                                                   USceneComponent* rail,
                                                   USceneComponent* cart,
                                                   USceneComponent* payload)
{
    CenterComponent = center;
    RailComponent = rail;
    CartComponent = cart;
    PayloadComponent = payload;
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
    // UE4 is in centimeters, crane model is in meters
    RailOffset = (float)(state.RailOffset * 100);
    CartOffset = (float)(state.CartOffset * 100);
    PayloadPosition = FVector{
        (float)(state.PayloadX * 100),
        (float)(state.PayloadY * 100),
        (float)(state.PayloadZ * 100)
    };
    auto text = Model->GetStateDebugText();
    GEngine->AddOnScreenDebugMessage(1, 5.0f, FColor::Red, FString::Printf(L"Frail: %.2f N", ForceRail));
    GEngine->AddOnScreenDebugMessage(2, 5.0f, FColor::Red, FString::Printf(L"Fcart: %.2f N", ForceCart));
    GEngine->AddOnScreenDebugMessage(3, 5.0f, FColor::Red, FString::Printf(L"Fwndg: %.2f N", ForceWinding));
    GEngine->AddOnScreenDebugMessage(4, 5.0f, FColor::Red, FString{text.c_str()});
}

void UCraneSimulationComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // update model with parameters
    // we do this every frame to allow full dynamic tweaking of
    // the crane while the game is running
    Model->Type = static_cast<crane3d::ModelType>(ModelType);
    Model->Mrail = crane3d::Mass{RailMass};
    Model->Mcart = crane3d::Mass{CartMass};
    Model->Mpayload = crane3d::Mass{PayloadMass};
    Model->G = Gravity;
    Model->g = crane3d::Accel{Gravity};

    Model->RailFriction = RailFriction;
    Model->CartFriction = CartFriction;
    Model->WindingFriction = WindingFriction;

    Model->RailLimitMin = RailLimitMin / 100.0f;
    Model->RailLimitMax = RailLimitMax / 100.0f;
    Model->CartLimitMin = CartLimitMin / 100.0f;
    Model->CartLimitMax = CartLimitMax / 100.0f;
    Model->LineLimitMin = LineLimitMin / 100.0f;
    Model->LineLimitMax = LineLimitMax / 100.0f;

    using crane3d::Force;
    crane3d::ModelState state = Model->Update(DeltaTime, Force{ForceRail}, Force{ForceCart}, Force{ForceWinding});
    UpdateVisibleFields(state);

    // reset all forces for this frame
    ForceRail = 0.0;
    ForceCart = 0.0;
    ForceWinding = 0.0;

    if (CenterComponent)
    {
        if (RailComponent)
        {
            FVector railPos = RailComponent->RelativeLocation;
            railPos.X = RailOffset;
            RailComponent->SetRelativeLocation(railPos);
        }

        if (CartComponent)
        {
            FVector cartPos = RailComponent->RelativeLocation;
            cartPos.X = RailOffset;
            cartPos.Y = CartOffset;
            CartComponent->SetRelativeLocation(cartPos);
        }

        if (PayloadComponent)
        {
            PayloadComponent->SetRelativeLocation(PayloadPosition);

            // cable visualization (DEBUG)
            DrawDebugLine(GetWorld(), CartComponent->GetComponentLocation(),
                PayloadComponent->GetComponentLocation(), FColor(15, 15, 15), false, -1, 0, 1);
        }
    }
}

