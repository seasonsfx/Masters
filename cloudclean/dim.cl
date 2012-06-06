__kernel void dim(__global float4* point, float anim)
{
  unsigned int x = get_global_id(0);

  // write output vertex
  //point[x].w = point[x].w; //*sin(anim);
}
