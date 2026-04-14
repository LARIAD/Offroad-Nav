import cupy as cp
from typing import List
from .plugin_manager import PluginBase
import cupyx.scipy.ndimage as ndimage


class Csf_from_pcl(PluginBase):
    def __init__(
        self,
        input_base_layer_name: str = "elevation",
        input_csf_layer_name: str = "elevation",
        safe_all: float = 0.05,
        memory: bool = True,
        cell_n: int = 8,
        layer_name: str = "Csf_from_pcl",
        **kwargs,
    ):
        super().__init__()
        self.cell_n=cell_n
        self.input_base_layer_name = input_base_layer_name
        self.input_csf_layer_name = input_csf_layer_name
        self.safe_all = cp.array([safe_all], dtype=cp.float32)
        self.last_local=cp.zeros((self.cell_n, self.cell_n), dtype=cp.float32)
        self.memory=memory
        self.layer_name=layer_name
        # self.flag_once=False

    def __call__(
        self,
        elevation_map: cp.ndarray,
        layer_names: List[str],
        plugin_layers: cp.ndarray,
        plugin_layer_names: List[str],
        semantic_map: cp.ndarray,
        semantic_layer_names: List[str],
        *args,
    ) -> cp.ndarray:
        # print("sementic_layer_map: ",semantic_layer_names)
        self_layer_data = self.get_layer_data(
            elevation_map,
            layer_names,
            plugin_layers,
            plugin_layer_names,
            semantic_map,
            semantic_layer_names,
            self.layer_name,
        )
        # print("layer_name: ",self.layer_name)
        # # idx = plugin_layer_names.index(name)
        # # self_layer=plugin_layers[idx]
        # if (len(args)==2):
        #     self_layer_data=cp.roll(self_layer_data,args[1],axis=(0,1))

        base_layer_data = self.get_layer_data(
            elevation_map,
            layer_names,
            plugin_layers,
            plugin_layer_names,
            semantic_map,
            semantic_layer_names,
            self.input_base_layer_name,
        )
        if base_layer_data is None:
            #print(f"No layers are found, using elevation!")
            base_layer_data = self.get_layer_data(
                elevation_map,
                layer_names,
                plugin_layers,
                plugin_layer_names,
                semantic_map,
                semantic_layer_names,
                "elevation",
            )


        
        csf_layer_data = self.get_layer_data(
            elevation_map,
            layer_names,
            plugin_layers,
            plugin_layer_names,
            semantic_map,
            semantic_layer_names,
            self.input_csf_layer_name,
        )
        if csf_layer_data is None:
            #print(f"No layers are found, using elevation!")
            csf_layer_data = self.get_layer_data(
                elevation_map,
                layer_names,
                plugin_layers,
                plugin_layer_names,
                semantic_map,
                semantic_layer_names,
                "elevation",
            )
        # layer_np = cp.asnumpy(layer_data)
        # print("shape-semmmmmmmmmmmm: ",semantic_map.shape)
        # print("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII")
        # print("I                                                                                       I")
        # print("I                                   ",self.input_csf_layer_name)
        # print("I                                  sementic_map:",base_layer_data[11,12])
        # print("I                                  elevation_map:",csf_layer_data[11,12])
        # print("I                                  moins:",base_layer_data[11,12]-csf_layer_data[11,12])
        # print("I                                                                                       I")
        # print("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII")
        if semantic_layer_names==[]:
            return self.last_local
           
        local=cp.where( (base_layer_data[:,:]>=-60.0) & (csf_layer_data[:,:]>=-60.0),base_layer_data[:,:]-csf_layer_data[:,:],cp.nan)
        
        if ( not (self_layer_data is None) and self.memory):
            mask=cp.full(local.shape,False)
            mask[1:-1,1:-1]=((cp.isnan(local[1:-1,:-2]))|(cp.isnan(local[1:-1,2:]))|(cp.isnan(local[:-2,1:-1]))|(cp.isnan(local[2:,1:-1])))&(~cp.isnan(local[1:-1,1:-1]))
            local[mask]=cp.where((local[mask]-self_layer_data[mask]<0.0),self_layer_data[mask],local[mask])
            # new=cp.zeros(local.shape)
            # new[mask]=12.
            # local=new
            # local[1:,:-2]=cp.where( cp.)
            local=cp.where( cp.isnan(local[:,:]),self_layer_data[:,:],local[:,:] )
        # local=((self_layer_data[:,:]>=0.0001) | (self_layer_data[:,:]<=-0.0001)) &
        self.last_local=local.copy()
        # local=  cp.where(semantic_map[1,:,:] <= -8000.,cp.nan,semantic_map[1,:,:])
        return local+self.safe_all #elevation_map[0,:,:]-
