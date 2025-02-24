using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using Unity.VisualScripting;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

	int[] 		Tet;
	int tet_number;			//The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();
	Vector3 g = new Vector3(0, -9.8f, 0);
	Vector3 floorPos = new Vector3(0, -3, 0);
	Vector3 floorNormal = new Vector3(0, 1, 0);
	float restitution = 0.5f;  // for collision, which is mu_N.
	float friction = 0.2f;  // mu_T.
	float smooth_blending = 0.5f;
	bool useBonus = false;

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

		//TODO: Need to allocate and assign inv_Dm
		inv_Dm = new Matrix4x4[tet_number];
		for (int e = 0; e < tet_number; ++e) {
			inv_Dm[e] = Build_Edge_Matrix(e).inverse;
		}
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
    	//TODO: Need to build edge matrix here.
        ret[0, 0] = X[Tet[tet * 4 + 1]].x - X[Tet[tet * 4]].x;
        ret[1, 0] = X[Tet[tet * 4 + 1]].y - X[Tet[tet * 4]].y;
        ret[2, 0] = X[Tet[tet * 4 + 1]].z - X[Tet[tet * 4]].z;
        
        ret[0, 1] = X[Tet[tet * 4 + 2]].x - X[Tet[tet * 4]].x;
        ret[1, 1] = X[Tet[tet * 4 + 2]].y - X[Tet[tet * 4]].y;
        ret[2, 1] = X[Tet[tet * 4 + 2]].z - X[Tet[tet * 4]].z;
        
        ret[0, 2] = X[Tet[tet * 4 + 3]].x - X[Tet[tet * 4]].x;
        ret[1, 2] = X[Tet[tet * 4 + 3]].y - X[Tet[tet * 4]].y;
        ret[2, 2] = X[Tet[tet * 4 + 3]].z - X[Tet[tet * 4]].z;

		ret[3, 3] = 1;
		return ret;
    }

	Matrix4x4 Mat_Sub_Mat(Matrix4x4 a, Matrix4x4 b) {
		Matrix4x4 ret = Matrix4x4.zero;
		for (int i = 0; i != 3; ++i) {
			for (int j = 0; j != 3; ++j) {
				ret[i, j] = a[i, j] - b[i, j];
			}
		}
		ret[3, 3] = 1;
		return ret;
	}

	Matrix4x4 Mat_Add_Mat(Matrix4x4 a, Matrix4x4 b) {
		Matrix4x4 ret = Matrix4x4.zero;
		for (int i = 0; i != 3; ++i) {
			for (int j = 0; j != 3; ++j) {
				ret[i, j] = a[i, j] + b[i, j];
			}
		}
		ret[3, 3] = 1;
		return ret;
	}

	Matrix4x4 Mat_Mul_Scalar(Matrix4x4 m, float c) {
		Matrix4x4 ret = Matrix4x4.zero;
		for (int i = 0; i != 3; ++i) {
			for (int j = 0; j != 3; ++j) {
				ret[i, j] = m[i, j] * c;
			}
		}
		ret[3, 3] = 1;
		return ret;
	}

	float Trace(Matrix4x4 m) {
		return m[0, 0] + m[1, 1] + m[2, 2];
	}

	float Trace2(Matrix4x4 m) {
		return m[0, 0] * m[0, 0] + m[1, 1] * m[1, 1] + m[2, 2] * m[2, 2];
	}

	float Trace4(Matrix4x4 m) {
		return Mathf.Pow(m[0, 0], 4) + Mathf.Pow(m[1, 1], 4) + Mathf.Pow(m[2, 2], 4);
	}

    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
    		//TODO: Add gravity to Force.
			// We use "=" to reset the force.
			Force[i] = mass * g;
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
			Matrix4x4 force = Matrix4x4.zero;
			//TODO: Deformation Gradient
			Matrix4x4 F = Build_Edge_Matrix(tet) * inv_Dm[tet];
			if (!useBonus) {
				//TODO: Green Strain
				Matrix4x4 G = Mat_Mul_Scalar(Mat_Sub_Mat(F.transpose * F, Matrix4x4.identity), 0.5f);
				//TODO: Second PK Stress
				Matrix4x4 S = Mat_Add_Mat(Mat_Mul_Scalar(G, 2 * stiffness_1), Mat_Mul_Scalar(Matrix4x4.identity, stiffness_0 * Trace(G)));
				//TODO: Elastic Force
				force = Mat_Mul_Scalar(F * S * inv_Dm[tet].transpose, -1 / inv_Dm[tet].determinant / 6);
			} else {
		        Matrix4x4 U = Matrix4x4.zero;
		        Matrix4x4 Sigma = Matrix4x4.zero;
		        Matrix4x4 V = Matrix4x4.zero;
		        Matrix4x4 P = Matrix4x4.zero;
		        svd.svd(F, ref U, ref Sigma, ref V);
				// Principal invariants.
				float lamb0 = Sigma[0, 0];
				float lamb1 = Sigma[1, 1];
				float lamb2 = Sigma[2, 2];
		        float Ic = lamb0 * lamb0 + lamb1 * lamb1 + lamb2 * lamb2;
		        float IIc = Mathf.Pow(lamb0, 4) + Mathf.Pow(lamb1, 4) + Mathf.Pow(lamb2, 4);
				// STVK energy. W = s0/8(Ic-3)^2 + s1/4(IIC-2IC+3).
		        float dWdIc = 0.25f * stiffness_0 * (Ic - 3) - 0.5f * stiffness_1;
		        float dWdIIc = 0.25f * stiffness_1;
		        float dIcdlamb0 = 2 * lamb0;
		        float dIcdlamb1 = 2 * lamb1;
		        float dIcdlamb2 = 2 * lamb2;
		        float dIIcdlamb0 = 4 * Mathf.Pow(lamb0, 3);
		        float dIIcdlamb1 = 4 * Mathf.Pow(lamb1, 3);
		        float dIIcdlamb2 = 4 * Mathf.Pow(lamb2, 3);
		        P[0, 0] = dWdIc * dIcdlamb0 + dWdIIc * dIIcdlamb0;
		        P[1, 1] = dWdIc * dIcdlamb1 + dWdIIc * dIIcdlamb1;
		        P[2, 2] = dWdIc * dIcdlamb2 + dWdIIc * dIIcdlamb2;
				P[3, 3] = 1;
		        // Elastic Force
		        force = Mat_Mul_Scalar(U * P * V.transpose * inv_Dm[tet].transpose, -1 / inv_Dm[tet].determinant / 6);
			}
			
			Force[Tet[tet*4]].x += -1 * (force[0,0] + force[0,1] + force[0,2]);
	        Force[Tet[tet*4]].y += -1 * (force[1,0] + force[1,1] + force[1,2]);
	        Force[Tet[tet*4]].z += -1 * (force[2,0] + force[2,1] + force[2,2]);
	        Force[Tet[tet*4+1]].x += force[0,0];
	        Force[Tet[tet*4+1]].y += force[1,0];
	        Force[Tet[tet*4+1]].z += force[2,0];
	        Force[Tet[tet*4+2]].x += force[0,1];
	        Force[Tet[tet*4+2]].y += force[1,1];
	        Force[Tet[tet*4+2]].z += force[2,1];
	        Force[Tet[tet*4+3]].x += force[0,2];
	        Force[Tet[tet*4+3]].y += force[1,2];
	        Force[Tet[tet*4+3]].z += force[2,2];
    	}

		Laplacian_Smoothing();
    	for(int i=0; i<number; i++)
    	{
    		//TODO: Update X and V here.
            V[i] += dt * Force[i] / mass;
            V[i] *= damp;
    		//TODO: (Particle) collision with floor.
			Vector3 xi = X[i];
			Vector3 vi = V[i];
			if (Vector3.Dot(xi - floorPos, floorNormal) < 0 && Vector3.Dot(vi, floorNormal) < 0) {
				Vector3 vN = Vector3.Dot(vi, floorNormal) * floorNormal;
				Vector3 vT = vi - vN;
				float alpha = Mathf.Max(1 - friction * (1 + restitution) * vN.magnitude / vT.magnitude, 0);
				V[i] = - restitution * vN + alpha * vT;
			}
            X[i] += dt * V[i];
    	}
    }

    void Laplacian_Smoothing() {
		for(int i = 0; i < number; i++)
	    {
		    V_sum[i] = new Vector3(0, 0, 0);
		    V_num[i] = 0;
	    }

	    for(int tet = 0; tet < tet_number; tet++)
	    {
		    Vector3 sum = V[Tet[tet*4]] + V[Tet[tet*4+1]] + V[Tet[tet*4+2]] + V[Tet[tet*4+3]];
		    V_sum[Tet[tet*4]] += sum - V[Tet[tet*4]];
		    V_sum[Tet[tet*4+1]] += sum - V[Tet[tet*4+1]];
		    V_sum[Tet[tet*4+2]] += sum - V[Tet[tet*4+2]];
		    V_sum[Tet[tet*4+3]] += sum - V[Tet[tet*4+3]];
		    V_num[Tet[tet*4]] += 3;
		    V_num[Tet[tet*4+1]] += 3;
		    V_num[Tet[tet*4+2]] += 3;
		    V_num[Tet[tet*4+3]] += 3;
	    }

	    for(int i = 0; i < number; i++)
	    {
		    V[i] = smooth_blending * V[i] + (1 - smooth_blending) * V_sum[i] / V_num[i];
	    }
	}

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
		// Here we collect all the triangles, including inner surface and duplicates.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }
}
