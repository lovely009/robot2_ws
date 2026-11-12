from pathlib import Path
import hashlib,json
from OCP.STEPControl import STEPControl_Reader
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Sphere,GeomAbs_Cylinder
p=Path(__file__).resolve().parents[2]
src=p/'full_body_robot/cad/全身装配体（不含头部）.STEP'
if not src.exists():src=p/'全身装配体（不含头部）.STEP'
r=STEPControl_Reader();r.ReadFile(str(src));r.TransferRoots();shape=r.OneShape();ex=TopExp_Explorer(shape,TopAbs_FACE);spheres={};cylinders={};n=0
while ex.More():
 face=TopoDS.Face_s(ex.Current());a=BRepAdaptor_Surface(face,True);kind=a.GetType();n+=1
 if kind==GeomAbs_Sphere:
  s=a.Sphere();c=s.Location();v={'center_m':[c.X()/1000,c.Y()/1000,c.Z()/1000],'radius_m':s.Radius()/1000};key=tuple(round(x,8) for x in [*v['center_m'],v['radius_m']]);spheres[key]=v
 elif kind==GeomAbs_Cylinder:
  s=a.Cylinder()
  if 2<s.Radius()<35:
   c=s.Location();d=s.Axis().Direction();v={'origin_m':[c.X()/1000,c.Y()/1000,c.Z()/1000],'axis':[d.X(),d.Y(),d.Z()],'radius_m':s.Radius()/1000};key=tuple(round(x,8) for x in [*v['origin_m'],*v['axis'],v['radius_m']]);cylinders[key]=v
 ex.Next()
out={'source':'Exact analytic BRep surfaces from STEP, transformed into assembly world frame','step_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'face_count':n,'spheres':list(spheres.values()),'cylinders':list(cylinders.values())};dst=p/'full_body_robot/cad/assembly_audit/step-analytic-features.json';dst.write_text(json.dumps(out,indent=2)+'\n');print(n,'faces',len(spheres),'spheres',len(cylinders),'cylinders',flush=True)
