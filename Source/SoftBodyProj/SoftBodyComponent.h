#pragma once

#include "CoreMinimal.h"
#include "ProceduralMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "SoftBodyComponent.generated.h"
/**
*
*/
struct FSBVertex // SoftBody Vertex
{
	FSBVertex(float Mass)
		: Mass(Mass)
		, MassInverse(1/Mass)
		, Position(0, 0, 0)
		, Velocity(0, 0, 0)
		, Estimate(0, 0, 0)
	{}

	FSBVertex(FVector position, float Mass)
		: Mass(Mass)
		, MassInverse(1/Mass)
		, Position(position)
		, Velocity(0, 0, 0)
		, Estimate(position)
	{}

	float Mass;
	float MassInverse;
	FVector Position;
	FVector Velocity;
	FVector Estimate;
};

struct FSBConstraint
{
	FSBConstraint(int32 Ind1, int32 Ind2, float Init)
		: Index({Ind1, Ind2})
		, Initial(Init)
	{}

	FSBConstraint(int32 Ind1, int32 Ind2, int32 Ind3, int32 Ind4, float Init)
		: Index({Ind1, Ind2, Ind3, Ind4})
		, Initial(Init)
	{}

	TArray<int32>Index; // Indeksy wierzcholkow
	float Stiffness; // sztywnosc
	float Initial; // wartosc poczatkowa
};

struct FCollConstraint
{
	FCollConstraint(int32 Ind, FVector Loc, FVector Norm)
		: Index(Ind)
		, Location(Loc)
		, Normal(Norm)
	{}

	int32 Index;
	FVector Location;
	FVector Normal;
	//float Stiffness;// zawsze rowne 1.0
};
/**
*
*/
UCLASS(hidecategories = (Object, LOD), meta = (BlueprintSpawnableComponent), ClassGroup = Rendering)
class SOFTBODYPROJ_API USoftBodyComponent : public UProceduralMeshComponent
{
	GENERATED_BODY()

public:
	USoftBodyComponent(const FObjectInitializer& PCIP);
	virtual void BeginPlay()override;
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "C++")
		UMaterialInterface* Material;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SoftBody", meta = (ClampMin = "3", UIMin = "3", UIMax = "64"))
		int32 NumWidth = 32;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SoftBody", meta = (ClampMin = "3", UIMin = "3", UIMax = "64"))
		int32 NumLength = 36;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SoftBody", meta = (ClampMin = "0.001"))
		float VertexMass = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SoftBody", meta = (UIMin = "0", UIMax = "1024"))
		float UVscale = 4;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SoftBody", meta = (ClampMin = "0", UIMin = "0", UIMax = "64"))
		int32 AnchorPoints = 5;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SoftBody")
		bool Collision = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SoftBody", meta = (ClampMin = "0.005", UIMin = "0.005", UIMax = "0.1"))
		float Substep = 0.02;//0.016667

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SoftBody", meta = (ClampMin = "1", UIMin = "1", UIMax = "100"))
		int32 SolverIterations = 10;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SoftBody", meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
		float StretchStiffness = 0.9;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SoftBody", meta = (ClampMin = "0.0", UIMin = "0.0", UIMax = "1.0"))
		float BendStiffness = 0.5;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SoftBody", meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
		float Damping = 0.01;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SoftBody")
		FVector Gravity = FVector(0.0, 0.0, -980.0);

	UFUNCTION(BlueprintCallable, Category = "SoftBody")
		void SetExternalForce(FVector Force);

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SoftBody") // == PRESENTACJA & TESTOWANIE == //
		bool Movement = false;

private:
	void CreateConstrains();
	void GenerateCollisionConstrains();
	float GetVertDistance(int32 Index1, int32 Index2);
	double GetTriangleAngleCos(int32 Index1, int32 Index2, int32 Index3, int32 Index4);
	double GetTriangleAngle(int32 Index1, int32 Index2, int32 Index3, int32 Index4);
	double dot(FVector V, FVector W);
	FVector cross(FVector V, FVector W);
	FVector PlaneNormal(FVector P1, FVector P2, FVector P3);

	void CreateSBMesh();
	void UpdateVertices(float DeltaTime);
	void PhysicsSubstep(float DeltaTime);
	void SolveConstraintStretch();
	void SolveConstraintBend();
	void SolveConstraintColl();


	TArray<FSBVertex> SBVertices;
	TArray<FSBConstraint> SBConstraintsStretch;
	TArray<FSBConstraint> SBConstraintsBend;
	TArray<FCollConstraint> SBConstraintsColl;

	FVector ExternalForce;

	TArray<FVector> MeshVertices;
	TArray<int32> MeshTriangles;
	TArray<FVector> MeshNormals;
	TArray<FVector2D> MeshUV0;
	TArray<FProcMeshTangent> MeshTangents;
	TArray<FLinearColor> MeshVertexColors;

	float TimeRemainder;

	////////////////////////////////////
	// == PRESENTACJA & TESTOWANIE == //
	////////////////////////////////////

	float Counter = 0.0;
	void Move(float DeltaTime);
};