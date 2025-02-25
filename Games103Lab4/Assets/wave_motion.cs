using UnityEngine;
using System.Collections;

public class wave_motion : MonoBehaviour 
{
	int size 		= 100;
	float rate 		= 0.005f;
	float gamma		= 0.004f;
	float damping 	= 0.99f;
	float[,] 	old_h;
	float[,]	low_h;
	float[,]	vh;
	float[,]	b;

	bool [,]	cg_mask;
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	bool 	tag=true;

	Vector3 	cube_v = Vector3.zero;
	Vector3 	cube_w = Vector3.zero;

	GameObject cube;
	Bounds cube_bounds_local;
	Renderer cube_renderer;
	Bounds cube_bounds_world;
	GameObject block;
	Bounds block_bounds_local;
	Renderer block_renderer;
	Bounds block_bounds_world;
	float dx = 0.1f;
	float dx2 = 0.01f;
	Vector3 origin;

	// Rigid bodies.
	Vector3 g = new Vector3(0, -9.8f, 0);
	float rho = 1f;
	float mass = 0.5f;
	Matrix4x4 I_ref;
	Matrix4x4 I_ref_inverse;

	float linear_decay	= 0.999f;
	float angular_decay	= 0.98f;
	// rate = dt^2Hg / dx^2, dx^2 = 0.01, 
	// Let suppose that H = 0.5, half the cube length.
	// dt^2 = rate*dx^2 / Hg, dt ~ 0.003.
	float dt = 0.005f;  // For better results.


	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();

		// Added.
		// Get cube and block. The bound is the AABB bound in the local space.
		cube = GameObject.Find("Cube");
		cube_bounds_local = cube.GetComponent<MeshFilter>().mesh.bounds;
		cube_bounds_world = cube.GetComponent<Renderer>().bounds;
		block = GameObject.Find("Block");
		block_bounds_local = block.GetComponent<MeshFilter>().mesh.bounds;
		block_bounds_world = block.GetComponent<Renderer>().bounds;
		origin = new Vector3(-size*0.05f, 0, -size*0.05f);
		/*
		// Check the bound if you like.
		Debug.Log("------------Block local bounds------------");
		Debug.Log("Center : " + block_bounds_local.center);
        Debug.Log("Size : " + block_bounds_local.size);
        Debug.Log("bound Minimum : " + block_bounds_local.min);
        Debug.Log("bound Maximum : " + block_bounds_local.max);
		Debug.Log("------------Block world bounds------------");
		Debug.Log("Center : " + block_bounds_world.center);
        Debug.Log("Size : " + block_bounds_world.size);
        Debug.Log("bound Minimum : " + block_bounds_world.min);
        Debug.Log("bound Maximum : " + block_bounds_world.max);
		// Check the vertices of the cube.
		Vector3[] cube_vert = cube.GetComponent<MeshFilter>().mesh.vertices;
		Debug.Log("vert num " + cube_vert.Length);
		for (int i = 0; i < cube_vert.Length; ++i) Debug.Log("vert " + cube_vert[i]);
		*/
		float m = mass / 8;
		Vector3[] cube_vert = cube.GetComponent<MeshFilter>().mesh.vertices;
		for (int i=0; i<8; i++) 
		{
			float diag=m*cube_vert[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*cube_vert[i][0]*cube_vert[i][0];
			I_ref[0, 1]-=m*cube_vert[i][0]*cube_vert[i][1];
			I_ref[0, 2]-=m*cube_vert[i][0]*cube_vert[i][2];
			I_ref[1, 0]-=m*cube_vert[i][1]*cube_vert[i][0];
			I_ref[1, 1]-=m*cube_vert[i][1]*cube_vert[i][1];
			I_ref[1, 2]-=m*cube_vert[i][1]*cube_vert[i][2];
			I_ref[2, 0]-=m*cube_vert[i][2]*cube_vert[i][0];
			I_ref[2, 1]-=m*cube_vert[i][2]*cube_vert[i][1];
			I_ref[2, 2]-=m*cube_vert[i][2]*cube_vert[i][2];
		}
		I_ref [3, 3] = 1;
		I_ref_inverse = Matrix4x4.Inverse(I_ref);

		Vector3[] X=new Vector3[size*size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i*size+j].x=i*0.1f-size*0.05f;
			X[i*size+j].y=0;
			X[i*size+j].z=j*0.1f-size*0.05f;
		}  // The plane is 10 x 10 in length, [-5, 4] x [-5, 4].

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i=0; i<size-1; i++)
		for (int j=0; j<size-1; j++)
		{
			T[index*6+0]=(i+0)*size+(j+0);
			T[index*6+1]=(i+0)*size+(j+1);
			T[index*6+2]=(i+1)*size+(j+1);
			T[index*6+3]=(i+0)*size+(j+0);
			T[index*6+4]=(i+1)*size+(j+1);
			T[index*6+5]=(i+1)*size+(j+0);
			index++;
		}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			low_h[i,j]=99999;
			old_h[i,j]=0;
			vh[i,j]=0;
		}
	}

	// A is tridiag{-1, 2, -1} in valid rows, the Laplacian operator.
	// Given x, compute Ax.
	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			Ax[i,j]=0;
			if(i!=0)		Ax[i,j]-=x[i-1,j]-x[i,j];
			if(i!=size-1)	Ax[i,j]-=x[i+1,j]-x[i,j];
			if(j!=0)		Ax[i,j]-=x[i,j-1]-x[i,j];
			if(j!=size-1)	Ax[i,j]-=x[i,j+1]-x[i,j];
		}
	}

	// <x, y>.
	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			ret+=x[i,j]*y[i,j];
		}
		return ret;
	}

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
		}

		float rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for(int k=0; k<128; k++)
		{
			if(rk_norm<1e-10f)	break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha=rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				x[i,j]   +=alpha*cg_p[i,j];
				cg_r[i,j]-=alpha*cg_Ap[i,j];
			}

			float _rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta=_rk_norm/rk_norm;
			rk_norm=_rk_norm;

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				cg_p[i,j]=cg_r[i,j]+beta*cg_p[i,j];
			}
		}

	}

	void Shallow_Wave(float[,] old_h, float[,] h, float [,] new_h)
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] X    = mesh.vertices;
		//Step 1:
		//TODO: Compute new_h based on the shallow wave model.
		for (int i = 0; i != size; ++i) {
			for (int j = 0; j != size; ++j) {
				new_h[i, j] = h[i, j] + damping * (h[i, j] - old_h[i, j]);
				if (i - 1 >= 0) new_h[i, j] += rate * (h[i - 1, j] - h[i, j]);
				if (i + 1 < size) new_h[i, j] += rate * (h[i + 1, j] - h[i, j]);
				if (j - 1 >= 0) new_h[i, j] += rate * (h[i, j - 1] - h[i, j]);
				if (j + 1 < size) new_h[i, j] += rate * (h[i, j + 1] - h[i, j]);
			}
		}
		for (int i = 0; i != size; ++i) {
			for (int j = 0; j != size; ++j) {
				old_h[i, j] = h[i, j];
				h[i, j] = new_h[i, j];
				low_h[i, j] = 99999;  // reinitialize.
			}
		}

		//Step 2: Block->Water coupling
		//TODO: for block 1, calculate low_h.
		block_bounds_world = block.GetComponent<Renderer>().bounds;
		Vector3 block_min = block_bounds_world.min - origin;
		Vector3 block_max = block_bounds_world.max - origin;
		int block_x_min = (int) Mathf.Clamp(Mathf.Ceil(block_min.x / dx), 0, size - 1);
		int block_z_min = (int) Mathf.Clamp(Mathf.Ceil(block_min.z / dx), 0, size - 1);
		int block_x_max = (int) Mathf.Clamp(Mathf.Floor(block_max.x / dx) + 1, 0, size - 1);
		int block_z_max = (int) Mathf.Clamp(Mathf.Floor(block_max.z / dx) + 1, 0, size - 1);
		for (int i = block_x_min; i != block_x_max; ++i) {
			for (int j = block_z_min; j != block_z_max; ++j) {
				Vector3 ray_pos_0 = block.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, -10, X[i * size + j].z));
				Vector3 ray_pos_1 = block.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, -9, X[i * size + j].z));
				Ray ray = new Ray(ray_pos_0, ray_pos_1 - ray_pos_0);
				float dist = 99999;
				if (block_bounds_local.IntersectRay(ray, out dist)) low_h[i, j] = -10 + dist;
			}
		}
		//TODO: then set up b and cg_mask for conjugate gradient.
		for (int i = 0; i != size; ++i) {
			for (int j = 0; j != size; ++j) {
				if (low_h[i, j] < h[i, j]) {
					cg_mask[i, j] = true;
					b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
				} else {
					cg_mask[i, j] = false;
					b[i, j] = 0;
					vh[i, j] = 0;
				}
			}
		}
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		Conjugate_Gradient(cg_mask, b, vh, block_x_min, block_x_max, block_z_min, block_z_max);

		//TODO: for block 2, calculate low_h.
		cube_bounds_world = cube.GetComponent<Renderer>().bounds;
		Vector3 cube_min = cube_bounds_world.min - origin;
		Vector3 cube_max = cube_bounds_world.max - origin;
		int cube_x_min = (int) Mathf.Clamp(Mathf.Ceil(cube_min.x / dx), 0, size - 1);
		int cube_z_min = (int) Mathf.Clamp(Mathf.Ceil(cube_min.z / dx), 0, size - 1);
		int cube_x_max = (int) Mathf.Clamp(Mathf.Floor(cube_max.x / dx) + 1, 0, size - 1);
		int cube_z_max = (int) Mathf.Clamp(Mathf.Floor(cube_max.z / dx) + 1, 0, size - 1);
		for (int i = cube_x_min; i != cube_x_max; ++i) {
			for (int j = cube_z_min; j != cube_z_max; ++j) {
				Vector3 ray_pos_0 = cube.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, -10, X[i * size + j].z));
				Vector3 ray_pos_1 = cube.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, -9, X[i * size + j].z));
				Ray ray = new Ray(ray_pos_0, ray_pos_1 - ray_pos_0);
				float dist = 99999;
				if (cube_bounds_local.IntersectRay(ray, out dist)) low_h[i, j] = -10 + dist;
			}
		}
		//TODO: then set up b and cg_mask for conjugate gradient.
		for (int i = 0; i != size; ++i) {
			for (int j = 0; j != size; ++j) {
				if (low_h[i, j] < h[i, j]) {
					cg_mask[i, j] = true;
					b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
				} else {
					cg_mask[i, j] = false;
					b[i, j] = 0;
					vh[i, j] = 0;
				}
			}
		}
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		Conjugate_Gradient(cg_mask, b, vh, cube_x_min, cube_x_max, cube_z_min, cube_z_max);
	
		//TODO: Diminish vh.
		for (int i = 0; i != size; ++i) {
			for (int j = 0; j != size; ++j) {
				if (cg_mask[i, j]) vh[i, j] *= gamma;
				else vh[i, j] = 0; 
			}
		}
		//TODO: Update new_h by vh.
		for (int i = 0; i != size; ++i) {
			for (int j = 0; j != size; ++j) {
				if (i - 1 >= 0) new_h[i, j] += rate * (vh[i - 1, j] - vh[i, j]);
				if (i + 1 < size) new_h[i, j] += rate * (vh[i + 1, j] - vh[i, j]);
				if (j - 1 >= 0) new_h[i, j] += rate * (vh[i, j - 1] - vh[i, j]);
				if (j + 1 < size) new_h[i, j] += rate * (vh[i, j + 1] - vh[i, j]);
			}
		}

		//Step 3
		//TODO: old_h <- h; h <- new_h;
		for (int i = 0; i != size; ++i) {
			for (int j = 0; j != size; ++j) {
				h[i, j] = new_h[i, j];
			}
		}

		//Step 4: Water->Block coupling.
		// More TODO here.
		// Calculate the buoyancy.
		Vector3 f = Vector3.zero;
		Vector3 tau = Vector3.zero;
		Vector3 t = cube.transform.position;
		Quaternion q = cube.transform.rotation;
		Matrix4x4 R = Matrix4x4.Rotate(q);
		Matrix4x4 I_inverse = R * I_ref_inverse * R.transpose;
		for (int i = cube_x_min; i != cube_x_max; ++i) {
			for (int j = cube_z_min; j != cube_z_max; ++j) {
				if (vh[i, j] > 0) {
					Vector3 Rri = new Vector3(X[i * size + j].x, low_h[i, j], X[i * size + j].z) - t;
					Vector3 fi = -rho * g * dx2 * vh[i, j];
					f += fi;
					tau += Vector3.Cross(Rri, fi);
				}
			}
		}
		cube_v += dt * (g + f / mass);
		cube_v *= linear_decay;
		cube_w += dt * I_inverse.MultiplyPoint3x4(tau);
		cube_w *= angular_decay;
		t += dt * cube_v;
		Quaternion dw = new Quaternion(cube_w[0] * dt / 2, cube_w[1] * dt / 2, cube_w[2] * dt / 2, 0);
		Quaternion dq = dw * q;
		q.x += dq.x;
		q.y += dq.y;
		q.z += dq.z;
		q.w += dq.w;
		q = q.normalized;

		cube.transform.position = t;
		cube.transform.rotation = q;
	}
	

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		float[,] new_h = new float[size, size];
		float[,] h     = new float[size, size];

		//TODO: Load X.y into h.
		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			h[i, j] = X[i * size + j].y;
		}

		if (Input.GetKeyDown ("r")) 
		{
			//TODO: Add random water.
			float h_random = Random.value;
			int i_random = Random.Range(1, size - 1);
			int j_random = Random.Range(1, size - 1);
			float h_minus = h_random / 8;
			for (int i = i_random - 1; i != i_random + 2; ++i) {
				for (int j = j_random - 1; j != j_random + 2; ++j) {
					h[i, j] -= h_minus;
				}
			}
			h[i_random, j_random] += h_random + h_minus;
		}
	
		for(int l=0; l<8; l++)
		{
			Shallow_Wave(old_h, h, new_h);
		}

		//TODO: Store h back into X.y and recalculate normal.
		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i * size + j].y = h[i, j];
		}
		mesh.vertices  = X;
		mesh.RecalculateNormals ();

		
	}
}
