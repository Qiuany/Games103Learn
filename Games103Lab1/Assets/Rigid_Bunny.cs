using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	Vector3 g           = new Vector3(0, -9.8f, 0);  // -y is downward.
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia
	Matrix4x4 I_ref_inverse;

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision, which is mu_N.
	float friction      = 0.2f;                 // mu_T.


	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
		I_ref_inverse = Matrix4x4.Inverse(I_ref);
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	Matrix4x4 Mat_Mul_Float(Matrix4x4 a, float b) {
		for(int i = 0; i < 4; ++i){
			for(int j = 0; j < 4; ++j) a[i, j] *= b; 
        }
		return a;
    }
	
	Matrix4x4 Mat_Sub_Mat(Matrix4x4 a, Matrix4x4 b) {
		for(int i = 0; i < 4; ++i){
			for(int j = 0; j < 4; ++j) a[i, j] -= b[i, j]; 
        }
		return a;
	}


	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		Vector3 x = transform.position;
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		Matrix4x4 I_inverse = R * I_ref_inverse * R.transpose;

		Vector3 rc = new Vector3(0, 0, 0);
		int count = 0;

		// Collision detection.
		for (int i=0; i<vertices.Length; i++) {
			Vector3 Rri = R.MultiplyPoint3x4(vertices[i]);
			Vector3 xi = x + Rri;
			Vector3 vi = v + Vector3.Cross(w, Rri);
			if (Vector3.Dot(xi - P, N) < 0 && Vector3.Dot(vi, N) < 0) {
				rc += vertices[i];
				count++;
			}
		}

		if (count > 0) {
			// Average to one point.
			rc /= count;
			Vector3 Rrc = R.MultiplyPoint3x4(rc);
			Vector3 vc = v + Vector3.Cross(w, Rrc);

			// Calculate v new.
			Vector3 vN = Vector3.Dot(vc, N) * N;
			Vector3 vT = vc - vN;
			float alpha = Mathf.Max(1 - friction * (1 + restitution) * vN.magnitude / vT.magnitude, 0);
			Vector3 vN_new = - restitution * vN;
			Vector3 vT_new = alpha * vT;
			Vector3 v_new = vN_new + vT_new;

			// Calculate the impluse j.
			Matrix4x4 Rrc_cross_matrix = Get_Cross_Matrix(Rrc);
			Matrix4x4 K = Mat_Sub_Mat(Mat_Mul_Float(Matrix4x4.identity, 1 / mass), Rrc_cross_matrix * I_inverse * Rrc_cross_matrix);
			K[3, 3] = 1;
			Vector3 j = K.inverse.MultiplyPoint3x4(v_new - vc);

			// Update v and w.
			v += j / mass;
			w += I_inverse.MultiplyPoint3x4(Vector3.Cross(Rrc, j));
		}
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}

		if (launched) {
			// Part I: Update velocities
			v += dt * g;
			v *= linear_decay;
			// w doesn't change with gravity as the only external force.
			w *= angular_decay;

			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

			// Part III: Update position & orientation
			//Update linear status
			Vector3 x = transform.position;
			x += dt * v;
			//Update angular status
			Quaternion q = transform.rotation;
			Quaternion dw = new Quaternion(w[0] * dt / 2, w[1] * dt / 2, w[2] * dt / 2, 0);
			Quaternion dq = dw * q;
			q.x += dq.x;
			q.y += dq.y;
			q.z += dq.z;
			q.w += dq.w;
			q = q.normalized;

			// Part IV: Assign to the object
			transform.position = x;
			transform.rotation = q;
			// Unity's MeshRenderer renders the object based on the mesh data in the MeshFilter (mesh.vertices). 
			// Although the script doesn't directly modify the mesh vertices, it indirectly affects rendering by 
			// changing transform.position and transform.rotation.
		}
	}
}
