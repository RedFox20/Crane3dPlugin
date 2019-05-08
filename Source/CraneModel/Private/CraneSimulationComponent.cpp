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
    CartPosition.X = (float)(state.RailOffset * 100);
    CartPosition.Y = (float)(state.CartOffset * 100);
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

void UCraneSimulationComponent::UpdateModelParameters()
{
    // update model with parameters
    // we do this every frame to allow full dynamic tweaking of
    // the crane while the game is running
    Model->SetType(static_cast<crane3d::ModelType>(ModelType));
    Model->Mrail = crane3d::Mass{RailMass};
    Model->Mcart = crane3d::Mass{CartMass};
    Model->Mpayload = crane3d::Mass{PayloadMass};
    Model->g = crane3d::Accel{Gravity};

    Model->Rail.LimitMin = RailLimitMin / 100.0f;
    Model->Rail.LimitMax = RailLimitMax / 100.0f;
    Model->Cart.LimitMin = CartLimitMin / 100.0f;
    Model->Cart.LimitMax = CartLimitMax / 100.0f;
    Model->Line.LimitMin = LineLimitMin / 100.0f;
    Model->Line.LimitMax = LineLimitMax / 100.0f;
}

void UCraneSimulationComponent::TickComponent(float DeltaTime,
    ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    UpdateModelParameters();

    using crane3d::Force;
    crane3d::ModelState state = Model->UpdateFixed(1.0/2000.0, DeltaTime,
                Force{ForceRail}, Force{ForceCart}, Force{ForceWinding});

    UpdateVisibleFields(state);
    UpdateVisibleComponents();

    // reset input forces for this frame
    ForceRail = ForceCart = ForceWinding = 0.0;
}

void UCraneSimulationComponent::UpdateVisibleComponents()
{
    if (!CenterComponent || !PayloadComponent || !CartComponent || !RailComponent)
        return;

    FVector railPos = RailComponent->RelativeLocation;
    FVector cartPos = RailComponent->RelativeLocation;
    railPos.X = CartPosition.X;
    cartPos.X = CartPosition.X;
    cartPos.Y = CartPosition.Y;
    RailComponent->SetRelativeLocation(railPos);
    CartComponent->SetRelativeLocation(cartPos);
    PayloadComponent->SetRelativeLocation(PayloadPosition);

    // rotate the payload towards cart
    FVector dir = CartComponent->GetComponentLocation()
                - PayloadComponent->GetComponentLocation();
    dir.Normalize();
    FRotator rot = dir.Rotation();
    rot.Pitch -= 90;
    PayloadComponent->RelativeRotation = rot;

    // cable line
    DrawDebugLine(GetWorld(), CartComponent->GetComponentLocation(),
        PayloadComponent->GetComponentLocation(),
        FColor(15, 15, 15), false, -1, 0, 1);
}

