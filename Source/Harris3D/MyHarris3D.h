// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <iostream>
#include <vector>
#include <set>
#include <fstream>

#include "KismetProceduralMeshLibrary.h"
#include "MeshDescription.h"
#include "MeshDescriptionBuilder.h"
#include "StaticMeshAttributes.h"
#include "StaticMeshResources.h"

#include "TimerManager.h"
#include "mesh.h"
#include "DrawDebugHelpers.h"


#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MyHarris3D.generated.h"



// ================================================================================================

UCLASS()
class HARRIS3D_API AMyHarris3D : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AMyHarris3D();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	UPROPERTY(EditAnywhere, Category="Inspector")
	int m_ringSize = 5;

	//UPROPERTY(EditAnywhere, Category="Inspector")
	//bool m_type = false;

	UPROPERTY(EditAnywhere, Category="Inspector")
	double m_fraction = 0.01;

	UPROPERTY(EditAnywhere, Category="Inspector")
	double m_k = 0.04;

	//UPROPERTY(EditAnywhere, Category="Inspector")
	//FColor m_color = FColor (255, 0, 0);

	//UPROPERTY(EditAnywhere, Category="Inspector")
	//double m_radius = 5;

	UPROPERTY (VisibleAnywhere)
	UStaticMeshComponent* m_pMeshCom;

	UPROPERTY(EditAnywhere, Category="Inspector")
	bool m_debugDraw = false;

	//UPROPERTY(EditAnywhere, Category="Inspector")
	//char* m_meshDir = ;
	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	int typeSelection;
	double fraction_constant;
	double k_parameter;
	
	std::vector<int> selectedVrts;
	std::vector<int> selectedVrts_clustering;
	
	TArray <FVector> selectedVrtLocs;
	TArray <FVector> selectedVrtLocs_clustering;

	TArray <FVector> selectedVrtNors;
	TArray <FVector> selectedVrtNors_clustering;
	
	UPROPERTY(BlueprintReadOnly, Category="Inspector")
	TArray <FVector> currentSelectedVrtLocs;
	TArray <FVector> currentSelectedVrtLocs_clustering;

	UPROPERTY(BlueprintReadOnly, Category="Inspector")
	TArray <FVector> currentSelectedVrtNors;
	TArray <FVector> currentSelectedVrtNors_clustering;
	
	FVector actorLocation;
	FVector actorScale;
	FRotator actorRotation;
	
	std::set<int> calculateNeighbourhood(int, std::vector <face>);
	std::set<int> calculateRingNeighbourhood(int); //neighbourhood considering k rings
	
	void calculateHarrisResponse();
	bool isLocalMaxima(unsigned int);

	void InitMyHarris3D ();

	mesh myMesh;

	int ringSize;
	std::vector<double> harrisRPoints;
	//UStaticMeshComponent* m_pMeshCom;

	//void DrawSelectedVertex ();
	void UpdateSelectedVertexLocation ();
	
	FVector GetVertexLocationByIndex (int i)
	{
		if (i >= currentSelectedVrtLocs.Num())
			return FVector (0, 0, 0);
		
		return  currentSelectedVrtLocs [i];
	}
	
	FVector GetVertexNormalByIndex (int i)
	{
		if (i >= currentSelectedVrtLocs.Num())
			return FVector (0, 0, 1);
		
		return  currentSelectedVrtNors [i];
	}
	
};
