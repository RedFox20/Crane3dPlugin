// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
// Distributed under MIT License
#include "CraneSimulationComponent.h"
#include "Engine/Engine.h"
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

static std::string to_string(ECraneModelType type)
{
    if (UEnum* uenum = FindObject<UEnum>(ANY_PACKAGE, TEXT("ECraneModelType"), true))
    {
        FString typeName = uenum->GetNameStringByValue((int64)type);
        return TCHAR_TO_ANSI(*typeName);
    }
    return {};
}

void UCraneSimulationComponent::BeginPlay()
{
    Super::BeginPlay();

    Model = std::make_unique<crane3d::Model>(to_string(ModelType));
    UpdateModelParameters();
    UpdateVisibleFields(Model->GetState());
}

void UCraneSimulationComponent::NextModelType()
{
    int next = (int)ModelType + 1;
    if (next > (int)ECraneModelType::NonLinearOriginal) next = 0;
    ModelType = (ECraneModelType)next;
}

void UCraneSimulationComponent::NextIntegrationMethod()
{
    int next = (int)IntegrationMethod + 1;
    if (next > (int)EIntegrationMethod::RK4) next = 0;
    IntegrationMethod = (EIntegrationMethod)next;
}

void UCraneSimulationComponent::UpdateVisibleFields(const crane3d::CraneState& state)
{
    // UE4 is in centimeters, crane model is in meters
    CartPosition.X = float(state.RailOffset * 100);
    CartPosition.Y = float(state.CartOffset * 100);
    PayloadPosition = FVector(state.PayloadX, state.PayloadY,state.PayloadZ) * 100;

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

    Model->SetCurrentModelByName(to_string(ModelType));
    Model->SetIntegrationMethod((crane3d::IntegrationMethod)IntegrationMethod);
}

void UCraneSimulationComponent::TickComponent(float DeltaTime,
    ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    UpdateModelParameters();

    double fixedTimeStep = 1.0 / IterationsPerSecond;
    using crane3d::Force;
    crane3d::CraneState state = Model->UpdateFixed(fixedTimeStep, DeltaTime,
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
    PayloadComponent->SetWorldRotation(rot);

    // cable line
    DrawDebugLine(GetWorld(), CartComponent->GetComponentLocation(),
        PayloadComponent->GetComponentLocation(),
        FColor(15, 15, 15), false, -1, 0, 1);
}

