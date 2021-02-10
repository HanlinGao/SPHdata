# SPHdata
Generate dataset with pySPlishSPlash

## Dependencies
pysplishsplash

pyvista (for reading vtk file)

## How to use
1. Choose one template.json from SPlishSPlash. (Could just copy from SPlishSPlasH/data/Scenes)
2. Put them under directory [template]
3. Also copy the models from SPlishSPlasH/data/models if needed, put them under directory [template/models] 
4. Modify the path in the RigidBodies part. (the path needs to point to the box or other models you want to use) 
5. Write one setting file as the example in the directory [settings]. Here you can specify the initial positions and velocity of the fluid block.
6. Name the final dataset and pass it as the third parameter. It will be created under the directory [datasets] 
