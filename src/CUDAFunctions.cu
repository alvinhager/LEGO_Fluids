#include "CUDAFunctions.h"

__device__ int  getFlatIndex(int i, int j, int k, int width, int height)
{
        return (unsigned int)i + (unsigned int)width * ((unsigned int)j + (unsigned int)height * (unsigned int)k);
};

__global__ void kernel6(float* velocity_field, 
                        float* weights, 
                        float* dx, FluidParticle* particles,
                        int *no_particles,
                        int* direction,
                        int* v_width, int* v_height, int* v_depth
                        )
{

  int U = 0;
  int V = 1;
  int W = 2;

    VectorMath::vec3 offset;
    float hdx = (float)(0.5 * (*dx));
    if (*direction == U)
        offset = VectorMath::vec3(0.0f, hdx, hdx);
    else if (*direction == V)
        offset = VectorMath::vec3(hdx, 0.0f, hdx);
    else if (*direction == W)
        offset = VectorMath::vec3(hdx, hdx, 0.0f);
    else
        return;

    float r = *dx;
    float rsq = r * r;
    float coef1 = (4.0f / 9.0f) * (1.0f / (r * r * r * r * r * r));
    float coef2 = (17.0f / 9.0f) * (1.0f / (r * r * r * r));
    float coef3 = (22.0f / 9.0f) * (1.0f / (r * r));

    // transfer particle velocity component to grid
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;

    int n=*no_particles;
    
    for (int pidx = index; pidx < n; pidx+=stride)
    {
     
        VectorMath::vec3 p = particles[pidx].position - offset;
        float velocityComponent = particles[pidx].velocity[*direction];

        GridIndex g = Grid3D::positionToGridIndex(p, *dx);
        GridIndex gmin((int)fmax(g.i - 1.0, 0.0), (int)fmax(g.j - 1.0, 0.0), (int)fmax(g.k - 1.0, 0.0));
        GridIndex gmax((int)fmin(g.i + 1.0, *v_width - 1.0),(int)fmin(g.j + 1.0, *v_height - 1.0), (int)fmin(g.k + 1.0, *v_depth - 1.0));

        for (int k = gmin.k; k <= gmax.k; k++)
        {
            for (int j = gmin.j; j <= gmax.j; j++)
            {
                for (int i = gmin.i; i <= gmax.i; i++)
                {
                    VectorMath::vec3 gpos = Grid3D::GridIndexToPosition(i, j, k, *dx);
                    VectorMath::vec3 v = gpos - p;
                    float distsq = v.x * v.x + v.y * v.y + v.z * v.z;
                    if (distsq < rsq)
                    {
                        float weight = 1.0f - coef1 * distsq * distsq * distsq + coef2 * distsq * distsq - coef3 * distsq;
                        int idx=getFlatIndex(i,j,k, *v_width, *v_height);
                        atomicAdd(&velocity_field[idx], weight*velocityComponent);
                        atomicAdd(&weights[idx], weight);
                    }
                }
            }
         }

      
     }

};


void computeVelocityFieldAndWeightsWithGPU(Array3D<float> &velocity_field,Array3D<bool> &isValueSet,int direction,FluidSimulation &fluidsim, Array3D<float> &weights )
{
 FluidParticle* h_particles= &fluidsim.particles[0];
 float *d_velocity_field, *d_weights, 
 
 float * d_dx;
 float dx=fluidsim.getdx();
 float* h_dx=&dx;

 int no_particles = fluidsim.particles.size();

 FluidParticle* d_particles;

 int *d_direction;
 bool *d_isValueSet;
 int* d_v_width, *d_v_height,*d_v_depth, 
 int* d_no_particles;

  cudaMalloc((void**)&d_velocity_field, sizeof(float)*velocity_field.getSize());
  cudaMemcpy(d_velocity_field, velocity_field.getRawArray(),sizeof(float)*velocity_field.getSize(), cudaMemcpyHostToDevice);

  cudaMalloc((void**)&d_weights, sizeof(float)*weights.getSize());
  cudaMemcpy(d_weights, weights.getRawArray(),sizeof(float)*weights.getSize(), cudaMemcpyHostToDevice);

  cudaMalloc((void**)&d_dx, sizeof(float));
  cudaMemcpy(d_dx, &dx, sizeof(float), cudaMemcpyHostToDevice);

  cudaMalloc((void**)&d_particles, sizeof(FluidParticle)*fluidsim.particles.size());
  cudaMemcpy(d_particles, h_particles, sizeof(FluidParticle)*fluidsim.particles.size(), cudaMemcpyHostToDevice);

  cudaMalloc((void**)&d_direction, sizeof(int));
  cudaMemcpy(d_direction, &direction, sizeof(int), cudaMemcpyHostToDevice);

  cudaMalloc((void**)&d_no_particles, sizeof(int));
  cudaMemcpy(d_no_particles,&no_particles, sizeof(int), cudaMemcpyHostToDevice);

  // dimensions of grid passed to GPU
  cudaMalloc((void**)&d_v_width, sizeof(int));
  cudaMemcpy(d_v_width, &velocity_field.width, sizeof(int), cudaMemcpyHostToDevice);

  cudaMalloc((void**)&d_v_height, sizeof(int));
  cudaMemcpy(d_v_height, &velocity_field.height, sizeof(int), cudaMemcpyHostToDevice);

  cudaMalloc((void**)&d_v_depth, sizeof(int));
  cudaMemcpy(d_v_depth, &velocity_field.depth, sizeof(int), cudaMemcpyHostToDevice);

int N=fluidsim.particles.size();
int blockSize=256; 
int numBlocks = (N+ blockSize - 1)/blockSize;

kernel6<<<blockSize,numBlocks>>>(d_velocity_field, d_weights, d_dx, d_particles, d_no_particles, d_direction, d_v_width,d_v_height, d_v_depth);

cudaDeviceSynchronize();

cudaMemcpy(velocity_field.getRawArray(), d_velocity_field, sizeof(float) * velocity_field.getSize(), cudaMemcpyDeviceToHost);
cudaMemcpy(weights.getRawArray(), d_weights, sizeof(float) * weights.getSize(), cudaMemcpyDeviceToHost);

// free device memory
cudaFree(d_velocity_field);
cudaFree(d_weights);
cudaFree(d_weights);
cudaFree(d_dx);
cudaFree(d_v_width);
cudaFree(d_v_height);
cudaFree(d_v_depth);
cudaFree(d_particles);
cudaFree(d_no_particles);
};
