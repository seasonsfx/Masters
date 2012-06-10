__kernel void dim(__global float4* point, float anim)
{
  unsigned int x = get_global_id(0);

  //int on = ((((int)anim*100)%2) -1);

  // write output vertex
  // point[x].w = sin(anim);

  // point[x].z = point[x].z + on;
}
