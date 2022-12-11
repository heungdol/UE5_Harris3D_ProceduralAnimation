#pragma once

UENUM(BlueprintType)
enum class EVertexType : uint8
{
	NONE
	, VERTEX_BUMP
	, VERTEX_FLAT 
	, VERTEX_SINK 
};

UENUM(BlueprintType)
enum class EVertexNormalType : uint8
{
	NONE 
	, VERTEX_UP 
	, VERTEX_PARALLEL 
	, VERTEX_DOWN 
};

UENUM(BlueprintType)
enum class EDetectorType : uint8
{
	NONE
	, DT_HR
	, DT_HKS
	, DT_ISS
	, DT_MS
};
