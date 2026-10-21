"""Fast filled-body static-map guard downstream of the velocity smoother."""
import math
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from .reservations import command_path


class StaticBodyGuard:
    def __init__(self,map_file,margin=0.,reaction=.35,decel=.35,angular_decel=.8):
        meta=yaml.safe_load(Path(map_file).read_text());image=Image.open(Path(map_file).parent/meta['image']).convert('L')
        values=np.asarray(image,dtype=float)/255.
        probability=values if meta.get('negate',0) else 1-values
        self.blocked=np.flipud(probability>=float(meta.get('free_thresh',.196)))
        self.res=float(meta['resolution']);self.ox,self.oy=map(float,meta['origin'][:2])
        self.height,self.width=self.blocked.shape
        self.margin=margin;self.reaction=reaction;self.decel=decel;self.angular_decel=angular_decel

    def free(self,pose,body):
        x,y,yaw=pose;l,w,padding=body
        c,s=math.cos(yaw),math.sin(yaw);hx=l/2+padding+self.margin;hy=w/2+padding+self.margin
        ax,ay=abs(c)*hx+abs(s)*hy,abs(s)*hx+abs(c)*hy;r=self.res
        x0,x1=math.floor((x-ax-self.ox)/r),math.floor((x+ax-self.ox)/r)
        y0,y1=math.floor((y-ay-self.oy)/r),math.floor((y+ay-self.oy)/r)
        if x0<0 or y0<0 or x1>=self.width or y1>=self.height:return False
        rows,cols=np.nonzero(self.blocked[y0:y1+1,x0:x1+1])
        if not len(rows):return True
        dx=self.ox+(cols+x0+.5)*r-x;dy=self.oy+(rows+y0+.5)*r-y
        cell=r/2*(abs(c)+abs(s))
        hit=(np.abs(dx)<=ax+r/2)&(np.abs(dy)<=ay+r/2)&(np.abs(c*dx+s*dy)<=hx+cell)&(np.abs(-s*dx+c*dy)<=hy+cell)
        return not bool(np.any(hit))

    def scale(self,pose,command,body):
        """Keep curvature, slow/stop if any future full body touches the map.

        Constant-twist prediction includes reaction plus half the conservative
        braking time (the displacement of a decelerating command). A further
        sampling allowance encloses corners between samples. No centre/corner-
        only collision shortcut, obstacle erasure or footprint shrinking.
        """
        if not all(math.isfinite(v) for v in (*pose,*command)):return 0.
        radius=math.hypot(body[0]/2+body[2],body[1]/2+body[2])
        step=self.res/8  # Match native swept_lattice.cpp exactly.
        enlarged=(body[0],body[1],body[2]+step/2)
        if not self.free(pose,enlarged):return 0.
        for scale in (1.,.75,.5,.25,.1,.05,.02,.01):
            velocity=tuple(v*scale for v in command)
            duration=self.reaction+.5*max(math.hypot(*velocity[:2])/self.decel,abs(velocity[2])/self.angular_decel)
            if all(self.free(p,enlarged) for p in command_path(pose,velocity,duration,radius,step)):
                return scale
        return 0.

    def filter(self,pose,command,body,measured=None):
        """Nearest safe twist among scaled requests and component projections.

        Never adds or reverses a requested component. Differential-drive inputs
        retain zero lateral velocity. All outputs pass the same filled-body
        braking sweep, including the straight slide around a convex corner.
        """
        # The drive still carries momentum after MPPI changes its request.
        # Check the observed braking sweep as well as the requested twist.
        # In particular, dropping requested omega must not hide actual rotation.
        if measured is not None and self.scale(pose,measured,body)<1.:
            return (0.,0.,0.)
        full=self.scale(pose,command,body)
        if full==1.:return tuple(command)
        radius=math.hypot(body[0]/2+body[2],body[1]/2+body[2])
        best=tuple(v*full for v in command)
        error=lambda v:(v[0]-command[0])**2+(v[1]-command[1])**2+(radius*(v[2]-command[2]))**2
        best_error=error(best)
        for mask in ((1,0,1),(0,1,1),(1,1,0),(1,0,0),(0,1,0),(0,0,1)):
            projected=tuple(v*m for v,m in zip(command,mask))
            if projected==command or not any(projected):continue
            scale=self.scale(pose,projected,body)
            result=tuple(v*scale for v in projected)
            value=error(result)
            if value<best_error:best,best_error=result,value
        return best
