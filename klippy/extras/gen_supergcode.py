# Generate supergcode
#
# Copyright (C) 2025-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, re

DEBUG_FLOG = 1
#0.04mm:635DPI

class SGcodeItemData:
    def __init__(self, coord_state, gtype, linestr):
        self.coord = coord_state
        self.gtype = gtype
        self.axis_flags = 0
        self.out_axis_flags = 0
        self.curpos = [0.0] * 8
        self.curmd = [0.0] *  8
        self.angle = 0.0
        self.mergeflag = 0
        self.pmergecnt = 0
        self.papprochvalue = 0
        self.instr = linestr

class GenSuperGcode:
    def __init__(self, config):
        self.printer = config.get_printer()
        #self.name = config.get_name().split(' ')[-1]
        self.epsilon = 0.001
        self.ratio_threshold = 0.01

        self.blocklen = config.getint('blocklen', 8, minval=2, maxval=24)
        self.enable = config.getint('enable', 1, minval=0, maxval=1)
        self.min_distancemm = config.getfloat('min_distance', 0.001, above=0.0)
        self.pmerge_max = config.getint('pmerge_max', 8, minval=0, maxval=32)
        self.AXIS_X = 0x1
        self.AXIS_Y = 0x2
        self.AXIS_Z = 0x4
        self.AXIS_A = 0x8
        self.AXIS_B = 0x10
        self.AXIS_C = 0x20
        self.PARAM_F = 0x40
        self.PARAM_S = 0x80    
        self.axis_map = [
            (self.AXIS_X, 'X', 0), (self.AXIS_Y, 'Y', 1),
            (self.AXIS_Z, 'Z', 2), (self.AXIS_A, 'A', 3),
            (self.AXIS_B, 'B', 4), (self.AXIS_C, 'C', 5)
        ]
        self.absolute_coord = True  # G90为True，G91为False   

        self.cached_cmds = []
        #self.path_points = []  # 存储路径点信息
        self.last_axis_flags = 0
        self.last_abs_pos = [-1.0] * 6
        self.abs_pos_bound = 0
        self.last_speed = 0.0
        self.last_power = 0.0
        #self.basenum_dist = 1000
        #self.basenum_dist = 255
        self.basenum_dist = 128
        self.outpowertabflag = 0
        #self.last_angle = 0.0
        self.gcode = self.printer.lookup_object('gcode')


    args_r = re.compile('([A-Z_]+|[A-Z*/])')
    def parse_gcode(self, line, coord_state, gtype):
        # 解析G0/G1指令
        itemdata = SGcodeItemData(coord_state, gtype, line)
        parts = self.args_r.split(line)
        numparts = len(parts)
        params = {parts[i]: float(parts[i+1].strip())
                  for i in range(1, numparts, 2)}
        #if line.startswith('G0') or line.startswith('G1'):
        for param, value in params.items():
            try:
                if param == 'X': 
                    itemdata.axis_flags |= self.AXIS_X
                    itemdata.out_axis_flags |= self.AXIS_X
                    if coord_state:
                        itemdata.curpos[0] = value
                        #if self.last_abs_pos[0] > self.abs_pos_bound:
                        itemdata.curmd[0] =  value - self.last_abs_pos[0]
                        self.last_abs_pos[0] = value
                    else:
                        itemdata.curmd[0] = value 
                    if abs(itemdata.curmd[0]) < self.min_distancemm:
                        itemdata.curmd[0] = 0.0
                        itemdata.axis_flags &= ~self.AXIS_X
                elif param == 'Y': 
                    itemdata.axis_flags |= self.AXIS_Y
                    itemdata.out_axis_flags |= self.AXIS_Y
                    if coord_state:
                        itemdata.curpos[1] = value
                        #if self.last_abs_pos[1] > self.abs_pos_bound:
                        itemdata.curmd[1] =  value - self.last_abs_pos[1]
                        self.last_abs_pos[1] = value                            
                    else:
                        itemdata.curmd[1] = value    
                    if abs(itemdata.curmd[1]) < self.min_distancemm:
                        itemdata.curmd[1] = 0.0
                        itemdata.axis_flags &= ~self.AXIS_Y                     
                elif param == 'Z': 
                    itemdata.axis_flags |= self.AXIS_Z
                    itemdata.out_axis_flags |= self.AXIS_Z
                    if coord_state:
                        itemdata.curpos[2] = value
                        #if self.last_abs_pos[2] > self.abs_pos_bound:
                        itemdata.curmd[2] =  value - self.last_abs_pos[2]
                        self.last_abs_pos[2] = value                            
                    else:
                        itemdata.curmd[2] = value                           
                    if abs(itemdata.curmd[2]) < self.min_distancemm:
                        itemdata.curmd[2] = 0.0
                        itemdata.axis_flags &= ~self.AXIS_Z
                elif param == 'A': 
                    itemdata.axis_flags |= self.AXIS_A
                    itemdata.out_axis_flags |= self.AXIS_A
                    if coord_state:
                        itemdata.curpos[3] = value
                        #if self.last_abs_pos[3] > self.abs_pos_bound:
                        itemdata.curmd[3] =  value - self.last_abs_pos[3]
                        self.last_abs_pos[3] = value                            
                    else:
                        itemdata.curmd[3] = value                           
                    if abs(itemdata.curmd[3]) < self.min_distancemm:
                        itemdata.curmd[3] = 0.0
                        itemdata.axis_flags &= ~self.AXIS_A
                elif param == 'B': 
                    itemdata.axis_flags |= self.AXIS_B
                    itemdata.out_axis_flags |= self.AXIS_B
                    if coord_state:
                        itemdata.curpos[4] = value
                        #if self.last_abs_pos[4] > self.abs_pos_bound:
                        itemdata.curmd[4] =  value - self.last_abs_pos[4]
                        self.last_abs_pos[4] = value                            
                    else:
                        itemdata.curmd[4] = value      
                    if abs(itemdata.curmd[4]) < self.min_distancemm:
                        itemdata.curmd[4] = 0.0
                        itemdata.axis_flags &= ~self.AXIS_B                     
                elif param == 'C': 
                    itemdata.axis_flags |= self.AXIS_C
                    itemdata.out_axis_flags |= self.AXIS_C
                    if coord_state:
                        itemdata.curpos[5] = value
                        #if self.last_abs_pos[5] > self.abs_pos_bound:
                        itemdata.curmd[5] =  value - self.last_abs_pos[5]
                        self.last_abs_pos[5] = value                            
                    else:
                        itemdata.curmd[5] = value   
                    if abs(itemdata.curmd[5]) < self.min_distancemm:
                        itemdata.curmd[5] = 0.0
                        itemdata.axis_flags &= ~self.AXIS_C                                                                        
                elif param == 'F': 
                    itemdata.axis_flags |= self.PARAM_F
                    itemdata.curmd[6] = value 
                    self.last_speed = value
                elif param == 'S': 
                    itemdata.axis_flags |= self.PARAM_S
                    itemdata.curmd[7] = value 
                    self.last_power = value
                    itemdata.papprochvalue = int(min(value/1000*255, 255))
            except ValueError:
                continue
        return itemdata



 
    def merge_gcode_in_cache(self):
        new_line = ""
        if not self.cached_cmds:
            return ""
        #todo
        curitem = self.cached_cmds[0]
        lastitem = self.cached_cmds[-1]
        abs_coord = curitem.coord
        max_index = curitem.curmd[:6].index(max(curitem.curmd[:6], key=abs))
        sums = [sum(item.curmd[i] for item in self.cached_cmds) for i in range(6)]
        dratiotab = [ round(min(item.curmd[max_index]/sums[max_index]*self.basenum_dist, self.basenum_dist)) for item in self.cached_cmds ]
        pvaltab   = [ int(min(item.curmd[7]/1000*255, 255)) for item in self.cached_cmds ]
        tpowerdistr = list(zip(dratiotab, pvaltab))
        ptablstr = "P" + ",".join(f"{dratio},{pval}" for dratio, pval in tpowerdistr)
        #lpowerdistr = [x for pair in zip(dratiotab, pvaltab) for x in pair]
        #ptablstr = "P" + ",".join(str(x) for x in lpowerdistr)
        if abs_coord:
            axis_parts = [
                f"{axis}{lastitem.curpos[idx]:.3f}"
                for flag, axis, idx in self.axis_map
                if curitem.out_axis_flags & flag
            ]
            # if curitem.axis_flags & self.PARAM_F:
            #     axis_parts.append(f"F{self.last_speed}")
            new_line = f"G{curitem.gtype} {' '.join(axis_parts)} {ptablstr}"
        else:
            axis_parts = [
                f"{axis}{sums[idx]:.3f}"
                for flag, axis, idx in self.axis_map
                if curitem.out_axis_flags & flag
            ]
            # if curitem.axis_flags & self.PARAM_F:
            #     axis_parts.append(f"F{self.last_speed}")
            new_line = f"G{curitem.gtype} {' '.join(axis_parts)} {ptablstr}"
        #new_line = f"{curitem.gtype} X{sums[0]:.3f} Y{sums[1]:.3f} Z{sums[2]:.3f} A{sums[3]:.3f} B{sums[4]:.3f} C{sums[5]:.3f} F{self.last_speed} S{self.last_power}"
        #new_line = f"{curitem.gtype} {curitem.instr}"
        return new_line


    def is_straight_line(self, itemdata):
        bstatus = False
        if itemdata.axis_flags & self.PARAM_F:
            return bstatus
        if not (itemdata.axis_flags & self.PARAM_S):
            return bstatus            
        if self.cached_cmds[-1].axis_flags != itemdata.axis_flags:
            return bstatus
        #todo 

        lastitem = self.cached_cmds[-1]
        all_same_direction = all(
            lastitem.curmd[i] * itemdata.curmd[i] > 0 
            for flag, _, i in self.axis_map 
            if itemdata.axis_flags & flag
        )
        if not all_same_direction:
            return bstatus

        base_idx = None
        base_ratio = None
        
        for flag, _, i in self.axis_map:
            if not (itemdata.axis_flags & flag):
                continue
            if abs(lastitem.curmd[i]) > self.epsilon:
                base_idx = i
                base_ratio = itemdata.curmd[i] / lastitem.curmd[i]
                break
        
        if base_idx is None:
            return bstatus

        for flag, _, i in self.axis_map:
            if not (itemdata.axis_flags & flag) or i == base_idx:
                continue
            if abs(lastitem.curmd[i]) <= self.epsilon:
                if abs(itemdata.curmd[i]) > self.epsilon:
                    return bstatus
                continue
            ratio = itemdata.curmd[i] / lastitem.curmd[i]
            if abs(ratio - base_ratio) > self.ratio_threshold:
                return bstatus
        bstatus = True
        return bstatus


    def check_flush_cache(self, absolute_coord_state):
        new_line = ""
        if self.cached_cmds:
            if len(self.cached_cmds) == 1:
                new_line = self.cached_cmds[0].instr + '\n'
            else:
                new_line = self.merge_gcode_in_cache() + '\n'
                self.outpowertabflag = 1
            self.cached_cmds = []
        return new_line

    def check_merge_power(self):
        pass
        return
        if self.pmerge_max == 0:
            return
        if len(self.cached_cmds) > 1:
            lastitem = self.cached_cmds[-1]
            prelastitem = self.cached_cmds[-2]
            if  prelastitem.mergeflag >0:
                return
            if lastitem.papprochvalue == prelastitem.papprochvalue:
                lastitem.pmergecnt  = prelastitem.pmergecnt + 1
                if lastitem.pmergecnt >= self.pmerge_max:
                    lastitem.mergeflag = 1
            else:
                prelastitem.mergeflag  = 1
        pass  


    def is_merge_gcode(self, itemdata):
        statuscode =  0
        new_line_str = ""
        if not self.cached_cmds:
            self.cached_cmds.append(itemdata)
            return statuscode, new_line_str

        if self.is_straight_line(itemdata):
            self.cached_cmds.append(itemdata)
            #statuscode = 0
            #self.check_merge_power()
            if len(self.cached_cmds) == self.blocklen:
                new_line_str = self.check_flush_cache(self.absolute_coord)
                if new_line_str:
                    statuscode = 2
            #return statuscode, new_line_str
        else:
            new_line_str = self.check_flush_cache(self.absolute_coord)
            self.cached_cmds.append(itemdata)
            #statuscode = 0
            if new_line_str:
                statuscode = 2            
            #return statuscode, new_line_str
        return statuscode, new_line_str


    def clean_cache_cmd(self):
        self.cached_cmds = []
        self.last_axis_flags = 0
        self.last_abs_pos = [-1.0] * 6
        self.last_speed = 0.0
        self.last_power = 0.0 
        self.outpowertabflag = 0       
        pass

    def debug_writefile(self, filename, content):
        if DEBUG_FLOG == 0:
            return
        if not content.endswith('\n'):
            content += '\n'            
        with open(filename, 'a') as f:
            f.write(content)
     
    def handle_supercode(self, line_gcode):
        retstatus = 0
        linestr = ""
        statuscode =  1
        if self.enable == 0:
            return statuscode, line_gcode

        linestr = line_gcode.strip().upper()
        if not linestr or linestr.startswith(';'):
            return statuscode, linestr
            
        if 'G90' in linestr:
            self.absolute_coord = True
            new_line = self.check_flush_cache(self.absolute_coord)
            if new_line:
                statuscode = 2
            return statuscode, new_line + linestr
        if 'G91' in linestr:
            self.absolute_coord = False
            new_line = self.check_flush_cache(self.absolute_coord)
            if new_line:
                statuscode = 2            
            return statuscode, new_line + linestr     

        if  linestr.startswith('G1'):
            itemdata = self.parse_gcode(linestr, self.absolute_coord, 1)
            statuscode, new_line = self.is_merge_gcode(itemdata)
            return statuscode,new_line
        elif  linestr.startswith('G0'):
            itemdata = self.parse_gcode(linestr, self.absolute_coord, 0)  
            new_line = self.check_flush_cache(self.absolute_coord)
            if new_line:
                statuscode = 2
            return statuscode, new_line + line_gcode
        else:               
            new_line = self.check_flush_cache(self.absolute_coord)
            if new_line:
                statuscode = 2
            return statuscode, new_line + line_gcode

    def get_status(self, eventtime=None):
        supergcode_status = {}
        supergcode_status['blocklen'] = self.blocklen
        supergcode_status['enable'] = self.enable   
        supergcode_status['min_distance'] = self.min_distancemm
        supergcode_status['last_abs_pos'] = self.last_abs_pos
        return dict(supergcode_status)


def load_config(config):
    return GenSuperGcode(config)
#self.gen_supergcode_obj = self.printer.lookup_object('gen_supergcode',None)
