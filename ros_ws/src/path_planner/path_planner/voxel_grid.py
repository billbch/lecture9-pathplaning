import numpy as np

class VoxelGrid:
    def __init__(self, size_x, size_y, size_z, resolution=0.5):
        self.resolution = resolution
        self.size_x = int(size_x / resolution)
        self.size_y = int(size_y / resolution)
        self.size_z = int(size_z / resolution)
        self.grid = np.zeros((self.size_x, self.size_y, self.size_z), dtype=np.uint8)
        self.no_fly_zones = []  # lista de (x_min, x_max, y_min, y_max, z_min, z_max)
        self.min_altitude = 1.0   # metros
        self.max_altitude = 20.0  # metros

    def world_to_grid(self, x, y, z):
        ix = int(x / self.resolution)
        iy = int(y / self.resolution)
        iz = int(z / self.resolution)
        return (ix, iy, iz)

    def grid_to_world(self, ix, iy, iz):
        x = ix * self.resolution
        y = iy * self.resolution
        z = iz * self.resolution
        return (x, y, z)

    def mark_obstacle(self, x, y, z, radius=1.0):
        """Marca una celda y su entorno como obstáculo"""
        cx, cy, cz = self.world_to_grid(x, y, z)
        r = int(radius / self.resolution)
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                for dz in range(-r, r+1):
                    nx, ny, nz = cx+dx, cy+dy, cz+dz
                    if self._in_bounds(nx, ny, nz):
                        self.grid[nx][ny][nz] = 1

    def add_no_fly_zone(self, x_min, x_max, y_min, y_max, z_min, z_max):
        self.no_fly_zones.append((x_min, x_max, y_min, y_max, z_min, z_max))
        # Marcar en el grid
        for x in np.arange(x_min, x_max, self.resolution):
            for y in np.arange(y_min, y_max, self.resolution):
                for z in np.arange(z_min, z_max, self.resolution):
                    ix, iy, iz = self.world_to_grid(x, y, z)
                    if self._in_bounds(ix, iy, iz):
                        self.grid[ix][iy][iz] = 2  # 2 = NFZ

    def is_free(self, ix, iy, iz):
        if not self._in_bounds(ix, iy, iz):
            return False
        # Restricción de altitud
        z_world = iz * self.resolution
        if z_world < self.min_altitude or z_world > self.max_altitude:
            return False
        return self.grid[ix][iy][iz] == 0

    def _in_bounds(self, ix, iy, iz):
        return (0 <= ix < self.size_x and
                0 <= iy < self.size_y and
                0 <= iz < self.size_z)