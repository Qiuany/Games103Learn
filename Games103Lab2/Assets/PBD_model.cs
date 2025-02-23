using UnityEngine;
using System.Collections;
using System.Linq.Expressions;

public class PBD_model: MonoBehaviour {

	float 		t= 0.0333f;
	float		damping= 0.99f;
	int[] 		E;  // Edge indices.
	float[] 	L;  // Edge lengths.
	Vector3[] 	V;  // Velocities.
	Vector3 g = new Vector3(0, -9.8f, 0);
	float mass = 1;


	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.
		int n=21;
		Vector3[] X  	= new Vector3[n*n];
		Vector2[] UV 	= new Vector2[n*n];
		int[] T	= new int[(n-1)*(n-1)*6];
		for(int j=0; j<n; j++)
		for(int i=0; i<n; i++)
		{
			X[j*n+i] =new Vector3(5-10.0f*i/(n-1), 0, 5-10.0f*j/(n-1));
			UV[j*n+i]=new Vector3(i/(n-1.0f), j/(n-1.0f));
		}
		int t=0;
		for(int j=0; j<n-1; j++)
		for(int i=0; i<n-1; i++)	
		{
			T[t*6+0]=j*n+i;
			T[t*6+1]=j*n+i+1;
			T[t*6+2]=(j+1)*n+i+1;
			T[t*6+3]=j*n+i;
			T[t*6+4]=(j+1)*n+i+1;
			T[t*6+5]=(j+1)*n+i;
			t++;
		}
		mesh.vertices	= X;
		mesh.triangles	= T;
		mesh.uv 		= UV;
		mesh.RecalculateNormals ();

		//Construct the original edge list
		int[] _E = new int[T.Length*2];
		for (int i=0; i<T.Length; i+=3) 
		{
			_E[i*2+0]=T[i+0];
			_E[i*2+1]=T[i+1];
			_E[i*2+2]=T[i+1];
			_E[i*2+3]=T[i+2];
			_E[i*2+4]=T[i+2];
			_E[i*2+5]=T[i+0];
		}  // 3 edges, with local indices (0, 1), (1, 2), (2, 0).
		//Reorder the original edge list
		for (int i=0; i<_E.Length; i+=2)
			if(_E[i] > _E[i + 1]) 
				Swap(ref _E[i], ref _E[i+1]);
		//Sort the original edge list using quicksort
		Quick_Sort (ref _E, 0, _E.Length/2-1); // Sorted edge pairs.

		int e_number = 0;
		for (int i=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
				e_number++;

		E = new int[e_number * 2];
		for (int i=0, e=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
			{
				E[e*2+0]=_E [i + 0];
				E[e*2+1]=_E [i + 1];
				e++;
			}

		L = new float[E.Length/2];
		for (int e=0; e<E.Length/2; e++) 
		{
			int i = E[e*2+0];
			int j = E[e*2+1];
			L[e]=(X[i]-X[j]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<X.Length; i++)
			V[i] = new Vector3 (0, 0, 0);
	}

	void Quick_Sort(ref int[] a, int l, int r)
	{
		int j;
		if(l<r)
		{
			j=Quick_Sort_Partition(ref a, l, r);
			Quick_Sort (ref a, l, j-1);
			Quick_Sort (ref a, j+1, r);
		}
	}

	int  Quick_Sort_Partition(ref int[] a, int l, int r)
	{
		int pivot_0, pivot_1, i, j;
		pivot_0 = a [l * 2 + 0];
		pivot_1 = a [l * 2 + 1];
		i = l;
		j = r + 1;
		while (true) 
		{
			do ++i; while( i<=r && (a[i*2]<pivot_0 || a[i*2]==pivot_0 && a[i*2+1]<=pivot_1));
			do --j; while(  a[j*2]>pivot_0 || a[j*2]==pivot_0 && a[j*2+1]> pivot_1);
			if(i>=j)	break;
			Swap(ref a[i*2], ref a[j*2]);
			Swap(ref a[i*2+1], ref a[j*2+1]);
		}
		Swap (ref a [l * 2 + 0], ref a [j * 2 + 0]);
		Swap (ref a [l * 2 + 1], ref a [j * 2 + 1]);
		return j;
	}

	void Swap(ref int a, ref int b)
	{
		int temp = a;
		a = b;
		b = temp;
	}

	void Strain_Limiting()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;
		Vector3[] X_sum = new Vector3[X.Length];
		int[] count = new int[X.Length];
		float blending = 0.2f;

		// Apply PBD here.
		// This is a Jacobi style PBD with linear blending at the end.
		// Distance constraints.
		for (int i = 0; i < L.Length; ++i) {
			int idx0 = E[i * 2 + 0];
			int idx1 = E[i * 2 + 1];
			Vector3 x01 = X[idx1] - X[idx0];
			float dist = x01.magnitude;
			Vector3 correction = 0.5f * (dist - L[i]) * x01 / dist;
			X_sum[idx0] += X[idx0] + correction;
			X_sum[idx1] += X[idx1] - correction;
			count[idx0]++;
			count[idx1]++;
		}

		for (int i = 0; i < X.Length; ++i) {
			// Dirichlet constraints.
			if (i == 0 || i == 20) continue;
			// Blending.
			Vector3 x_new = (X_sum[i] + blending * X[i]) / (count[i] + blending);
			V[i] += (x_new - X[i]) / t;
			X[i] = x_new;
		}
		mesh.vertices = X;
	}

	void Collision_Handling()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;
		GameObject sphere = GameObject.Find("Sphere");
        Vector3 center = sphere.transform.position;
		float r = 2.7f;
		float eps = 0.01f;
		
		//Handle colllision.
		for (int i = 0; i < X.Length; ++i) {
			if (i == 0 || i == 20) continue;
			float dist = (X[i] - center).magnitude;
			if (dist < r + eps) {
				Vector3 collided_point = center + (r + eps) * (X[i] - center) / dist;
				V[i] += (collided_point - X[i]) / t;
				X[i] = collided_point;
			}
		}
		mesh.vertices = X;
	}

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;

		for(int i=0; i<X.Length; i++)
		{
			if(i==0 || i==20)	continue;
			//Initial Setup
			V[i] *= damping;
			V[i] += t * g;
			X[i] += t * V[i];
		}
		mesh.vertices = X;  // The temp position p is already recorded in mesh.

		for(int l=0; l<32; l++)
			Strain_Limiting ();

		Collision_Handling ();

		mesh.RecalculateNormals ();

	}


}

