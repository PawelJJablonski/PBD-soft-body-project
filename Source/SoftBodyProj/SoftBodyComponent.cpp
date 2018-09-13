#include "SoftBodyComponent.h"
#include "WorldCollision.h"

USoftBodyComponent::USoftBodyComponent(const FObjectInitializer& PCIP)
	: Super(PCIP)
{
	//TimeRemainder = 0.0;

	PrimaryComponentTick.bCanEverTick = true;
	bTickInEditor = true;
	bAutoActivate = true;
	bAutoRegister = true; 
	PrimaryComponentTick.bStartWithTickEnabled = true; 
	PrimaryComponentTick.TickGroup = TG_PrePhysics; 
	SetMobility(EComponentMobility::Movable); 
	SetupActorComponentTickFunction(&PrimaryComponentTick); 
}

void USoftBodyComponent::BeginPlay()
{
	SetCollisionResponseToAllChannels(ECR_Overlap);

	for (int j=0; j<NumLength; j++) {
		for (int i=0; i<NumWidth; i++) {
			SBVertices.Add(FSBVertex(FVector(0, i * 20, j*-20),VertexMass));

			float temp = UVscale / (NumWidth - 1);
			MeshUV0.Add(FVector2D(i*temp, j*temp));
			
			MeshNormals.Add(FVector(-1, 0, 0));
			MeshTangents.Add(FProcMeshTangent(0, 1, 0));
			MeshVertexColors.Add(FLinearColor(0.75, 0.75, 0.75, 1.0));
		}
	}

	float step = 1; // zaczepy
	int32 ap;
	if (AnchorPoints >= NumWidth*0.5) {
		ap = 2;
	}
	else {
		ap = AnchorPoints;
	}
	if(ap>1)step = NumWidth / (ap-1);
	for (int i=0; i<ap; i++) {
		if(i==0)SBVertices[int32(step*i)].MassInverse = 0;
		else SBVertices[int32(step*i-1)].MassInverse = 0;
	}

	CreateConstrains();

	SetMaterial(0, Material);
	CreateSBMesh();
}

void USoftBodyComponent::CreateConstrains()
{
	for (int j = 0; j < NumLength-1; j++) {
		for (int i = 0; i < NumWidth-1; i++) {

			int rows = j*NumWidth;

			SBConstraintsStretch.Add(FSBConstraint(i+rows, i+rows+1, GetVertDistance(i+rows, i+rows+1)));
			SBConstraintsStretch.Add(FSBConstraint(i+rows, i+rows+NumWidth, GetVertDistance(i+rows, i+rows+NumWidth)));
			SBConstraintsStretch.Add(FSBConstraint(i+rows, i+rows+NumWidth+1, GetVertDistance(i+rows, i+rows+NumWidth+1)));
			SBConstraintsStretch.Add(FSBConstraint(i+rows+1, i+rows+NumWidth, GetVertDistance(i+rows+1, i+rows+NumWidth)));

			if (j == NumLength - 2) {
				SBConstraintsStretch.Add(FSBConstraint(i + rows + NumWidth, i + rows + NumWidth + 1, GetVertDistance(i + rows + NumWidth, i + rows + NumWidth + 1)));//koniec poziom
			}
			SBConstraintsBend.Add(FSBConstraint(i + rows, i + rows + NumWidth + 1, i + rows + NumWidth, i + rows + 1, GetTriangleAngle(i + rows, i + rows + NumWidth + 1, i + rows + NumWidth, i + rows + 1)));
			SBConstraintsBend.Add(FSBConstraint(i + rows + 1, i + rows + NumWidth, i + rows, i + rows + NumWidth + 1, GetTriangleAngle(i + rows + 1, i + rows + NumWidth, i + rows, i + rows + NumWidth + 1)));

			if (i > 0) {
				SBConstraintsBend.Add(FSBConstraint(i + rows, i + rows + NumWidth, i + rows - 1, i + rows + NumWidth + 1, GetTriangleAngle(i + rows, i + rows + NumWidth, i + rows - 1, i + rows + NumWidth + 1)));//pion
			}
			if (j > 0) {
				SBConstraintsBend.Add(FSBConstraint(i + rows, i + rows + 1, i + rows + NumWidth + 1, i + rows - NumWidth, GetTriangleAngle(i + rows, i + rows + 1, i + rows + NumWidth + 1, i + rows - NumWidth)));//poziom
			}
		}
		SBConstraintsStretch.Add(FSBConstraint(j*NumWidth+NumWidth-1, (j+1)*NumWidth+NumWidth-1, GetVertDistance(j*NumWidth+NumWidth-1, (j+1)*NumWidth+NumWidth-1)));//koniec pion
	}
}

void USoftBodyComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	
	if(Testing || Movement)Counter += DeltaTime;
	if(Movement)Move(DeltaTime);

	if (SBVertices.Num() > 3) // sprawdzenie, czy istnieje wystarczaj¹co wiele wierzcholkow
	{
		UpdateVertices(DeltaTime); // symulacja
		CreateSBMesh(); // rendering
	}
}

void USoftBodyComponent::UpdateVertices(float DeltaTime)
{
	TimeRemainder += DeltaTime;

	while(TimeRemainder>Substep){
		TimeRemainder -= Substep;
		PhysicsSubstep(Substep);
	}
}

void USoftBodyComponent::PhysicsSubstep(float DeltaTime)
{
	for (int i = 0; i < SBVertices.Num(); i++){
		if (SBVertices[i].MassInverse > 0){
			SBVertices[i].Velocity += DeltaTime * (Gravity + ExternalForce);
			SBVertices[i].Velocity *= 1.0-Damping;
			SBVertices[i].Estimate = SBVertices[i].Position + DeltaTime * SBVertices[i].Velocity;
		}
		SBVertices[i].Estimate = SBVertices[i].Position + DeltaTime * SBVertices[i].Velocity;
	}

	if(Collision)GenerateCollisionConstrains();

	for (int i = 0; i < SolverIterations; i++){
		SolveConstraintStretch();
		SolveConstraintBend();
		if (Collision)SolveConstraintColl();
	}

	if(Collision)SBConstraintsColl.Empty(); // usuwanie wiezow kolizji po ka¿dej iteracji
	
	for (int i = 0; i < SBVertices.Num(); i++){
		SBVertices[i].Velocity = (SBVertices[i].Estimate - SBVertices[i].Position) / DeltaTime;
		SBVertices[i].Position = SBVertices[i].Estimate;
	}
	////////////////////////// TESTOWANIE ///////////////////
	if(Testing && !flag){
		
		FVector temp = FVector(0.0,0.0,0.0);
		for(int i = 0; i < SBVertices.Num(); i++){
			temp += SBVertices[i].Velocity*SBVertices[i].Mass;//badanie zachowania pedu
		}

		float data = temp.Size();

		FString x = FString::SanitizeFloat(Counter);
		FString y = FString::SanitizeFloat(data);
		Lines.Add(FString(x+' '+y));
		
		if (Counter > 10.0f) {
			FString FolderDirectory = "C:/SoftBodyProj/test";
			FString FullDirectory = "C:/SoftBodyProj/test/Dane.txt";

			WriteDataToFile(FolderDirectory, FullDirectory,Lines);
			Lines.Empty();
			flag = true;
		}
	}
	//////////////////////////

}

void USoftBodyComponent::SolveConstraintStretch()
{
	for(int i=0; i<SBConstraintsStretch.Num(); i++)
	{
		FSBVertex& A = SBVertices[SBConstraintsStretch[i].Index[0]];
		FSBVertex& B = SBVertices[SBConstraintsStretch[i].Index[1]];
		float l0 = SBConstraintsStretch[i].Initial;

		FVector Temp1 = StretchStiffness*(A.MassInverse / (A.MassInverse + B.MassInverse)) * (((A.Estimate - B.Estimate).Size()) - l0) * ((A.Estimate - B.Estimate) / ((A.Estimate - B.Estimate).Size()));
		FVector Temp2 = StretchStiffness*(B.MassInverse / (A.MassInverse + B.MassInverse)) * (((A.Estimate - B.Estimate).Size()) - l0) * ((A.Estimate - B.Estimate) / ((A.Estimate - B.Estimate).Size()));

		A.Estimate -= Temp1;
		B.Estimate += Temp2;
	}
}

void USoftBodyComponent::SolveConstraintBend()
{
	for (int i = 0; i < SBConstraintsBend.Num(); i++)
	{
		FSBVertex& P1 = SBVertices[SBConstraintsBend[i].Index[0]];
		FSBVertex& P2 = SBVertices[SBConstraintsBend[i].Index[1]];
		FSBVertex& P3 = SBVertices[SBConstraintsBend[i].Index[2]];
		FSBVertex& P4 = SBVertices[SBConstraintsBend[i].Index[3]];

		double phi0 = SBConstraintsBend[i].Initial;

		FVector p2 = P2.Estimate - P1.Estimate;
		FVector p3 = P3.Estimate - P1.Estimate;
		FVector p4 = P4.Estimate - P1.Estimate;

		FVector n1 = cross(p2, p3) / cross(p2, p3).Size();
		FVector n2 = cross(p2, p4) / cross(p2, p4).Size();
		
		double d = FMath::Clamp(dot(n1, n2), -1.0, 1.0);
		double ang = acos(d);
		double C = ang - phi0;
		float scale = 5 * p4.Size();

		FVector q3 = ((cross(p2, n2) + cross(n1, p2)*d) / cross(p2, p3).Size());
		FVector q4 = ((cross(p2, n1) + cross(n2, p2)*d) / cross(p2, p4).Size());
		FVector q2 = (((cross(p3, n2) + cross(n1, p3)*d) / cross(p2, p3).Size()) - ((cross(p4, n1) + cross(n2, p4)*d) / cross(p2, p4).Size()));
		FVector q1 = ((-1)*q2 - q3 - q4);

		double s = FMath::Clamp(BendStiffness / (((P1.MassInverse*q1.SizeSquared()) + (P2.MassInverse*q2.SizeSquared()) + (P3.MassInverse*q3.SizeSquared()) + (P4.MassInverse*q4.SizeSquared()))*scale), -1e12f, 1e12f);
		
		FVector deltaP1 = - s * P1.MassInverse * sqrt(1 - d*d) * C * q1;
		FVector deltaP2 = - s * P2.MassInverse * sqrt(1 - d*d) * C * q2;
		FVector deltaP3 = - s * P3.MassInverse * sqrt(1 - d*d) * C * q3;
		FVector deltaP4 = - s * P4.MassInverse * sqrt(1 - d*d) * C * q4;

		P1.Estimate += deltaP1;
		P2.Estimate += deltaP2;
		P3.Estimate += deltaP3;
		P4.Estimate += deltaP4;
	}
}

void USoftBodyComponent::GenerateCollisionConstrains()
{
	if (!GetWorld() || GetCollisionEnabled() == ECollisionEnabled::NoCollision)return;

	FCollisionResponseContainer ResponseContainer(ECR_Block);
	FHitResult HitResult;

	SetCollisionResponseToAllChannels(ECR_Block);
	for (int i = 0; i < SBVertices.Num(); i++) {
		if (SBVertices[i].MassInverse > 0 && GetWorld()->SweepSingleByChannel(HitResult, SBVertices[i].Position + GetComponentLocation(), SBVertices[i].Estimate + GetComponentLocation(), FQuat::Identity, ECC_PhysicsBody, FCollisionShape::MakeSphere(5.0), FCollisionQueryParams(FName(), false, GetOwner()), FCollisionResponseParams(GetCollisionResponseToChannels()))) {

			FVector q;
			FVector n;
			
			if (!HitResult.bStartPenetrating)
			{
				q = HitResult.Location - GetComponentLocation();
				n = HitResult.Normal;
			}
			else // jesli CCD zawiedzie
			{
				q = HitResult.Location + (HitResult.Normal * HitResult.PenetrationDepth) - GetComponentLocation();
				n = HitResult.Normal;
			}
			SBConstraintsColl.Add(FCollConstraint(i, q, n));
		}
	}
	SetCollisionResponseToAllChannels(ECR_Overlap);

	return;
}

void USoftBodyComponent::SolveConstraintColl()
{
	for (int i=0; i<SBConstraintsColl.Num(); i++)
	{
		FSBVertex& Vert = SBVertices[SBConstraintsColl[i].Index];

		FVector Temp = (Vert.Estimate - SBConstraintsColl[i].Location).Size() * SBConstraintsColl[i].Normal;

		Vert.Estimate += Temp/SolverIterations;
	}
	return;
}

/////////////////////////////////////////////////////////////////////////
// -------------------------  POMOCNICZE  ---------------------------- //
/////////////////////////////////////////////////////////////////////////

float USoftBodyComponent::GetVertDistance(int32 Index1, int32 Index2)
{
	return (SBVertices[Index1].Position - SBVertices[Index2].Position).Size();
}

double USoftBodyComponent::GetTriangleAngle(int32 Index1, int32 Index2, int32 Index3, int32 Index4)
{
	float temp = FMath::Clamp(dot(PlaneNormal(SBVertices[Index1].Position, SBVertices[Index3].Position, SBVertices[Index2].Position), PlaneNormal(SBVertices[Index1].Position, SBVertices[Index4].Position, SBVertices[Index2].Position)), -1.0, 1.0);

	return acos(temp);
}

double USoftBodyComponent::dot(FVector V, FVector W)
{
	return V[0]*W[0] + V[1]*W[1] + V[2]*W[2];
}

FVector USoftBodyComponent::cross(FVector V, FVector W)
{
	return FVector(V[1]*W[2]-V[2]*W[1], V[2]*W[0]-V[0]*W[2], V[0]*W[1]-V[1]*W[0]);
}

FVector USoftBodyComponent::PlaneNormal(FVector P1, FVector P2, FVector P3)
{
	FVector U = P2-P1;
	FVector V = P3-P1;
	FVector N = cross(U, V);
	N.Normalize();

	return N;
}

void USoftBodyComponent::SetExternalForce(FVector Force)
{
	ExternalForce = Force;
}

/////////////////////////////////////////////////////////////////////////
// -----------------------  MESH  RENDERING  ------------------------- //
/////////////////////////////////////////////////////////////////////////

void USoftBodyComponent::CreateSBMesh()
{
	MeshVertices.Empty();
	MeshTriangles.Empty();

	for (int i = 0; i < SBVertices.Num(); i++) {
		MeshVertices.Add(SBVertices[i].Position);
	}

	for (int j = 0; j < NumLength - 1; j++) {
		for (int i = 0; i < NumWidth - 1; i++) {

			MeshTriangles.Add(i + (j*NumWidth));
			MeshTriangles.Add(i + (j*NumWidth) + 1 + NumWidth);
			MeshTriangles.Add(i + (j*NumWidth) + 1);

			MeshTriangles.Add(i + (j*NumWidth));
			MeshTriangles.Add(i + (j*NumWidth) + NumWidth);
			MeshTriangles.Add(i + (j*NumWidth) + 1 + NumWidth);
		}
	}

	CreateMeshSection_LinearColor(0, MeshVertices, MeshTriangles, MeshNormals, MeshUV0, MeshVertexColors, MeshTangents, true);
}

/////////////////////////////////////////////////////////////////////////
// -------------------  PREZENTACJA I TESTOWANIE  -------------------- //
/////////////////////////////////////////////////////////////////////////

void USoftBodyComponent::Move(float DeltaTime)
{
	float delta = 250.0 * DeltaTime;
	if ((FMath::FloorToInt(Counter) % 12) > 5)delta *= -1;
	for (int i = 0; i < SBVertices.Num(); i++) {
		if (SBVertices[i].MassInverse == 0)
		{
			SBVertices[i].Position += FVector(delta, 0.0, 0.0);
		}
	}
}

bool USoftBodyComponent::WriteDataToFile(FString FolderDirectory, FString FullDirectory, TArray<FString>Lines)
{
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	if (!PlatformFile.DirectoryExists(*FolderDirectory)){
		PlatformFile.CreateDirectory(*FolderDirectory);
	}
	return FFileHelper::SaveStringArrayToFile(Lines, *FullDirectory);
}