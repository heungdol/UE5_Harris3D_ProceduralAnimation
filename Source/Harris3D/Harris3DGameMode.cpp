// Copyright Epic Games, Inc. All Rights Reserved.

#include "Harris3DGameMode.h"
#include "Harris3DCharacter.h"
#include "UObject/ConstructorHelpers.h"

AHarris3DGameMode::AHarris3DGameMode()
{
	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/ThirdPersonCPP/Blueprints/ThirdPersonCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}
