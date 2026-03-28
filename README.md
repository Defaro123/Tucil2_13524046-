# Tucil2_13524046  
Voxelization Objek 3D Menggunakan Octree dan Divide and Conquer


## Penjelasan Singkat Program

Program ini dibuat untuk melakukan **voxelization** objek 3D dari file `.obj` menggunakan struktur data **Octree** dan pendekatan **divide and conquer**.
Objek 3D direpresentasikan sebagai kumpulan segitiga (mesh). Program akan membungkus objek dalam sebuah kubus awal, kemudian membagi kubus tersebut secara rekursif menjadi 8 bagian (Octree). Setiap sub-kubus diperiksa apakah beririsan dengan permukaan objek.
Jika sebuah kubus tidak beririsan, maka tidak akan diproses lebih lanjut (pruning). Jika masih beririsan dan sudah mencapai kedalaman maksimum, maka kubus tersebut akan dianggap sebagai **voxel**.
Hasil akhir berupa file `.obj` baru yang terdiri dari kumpulan kubus kecil (voxel) yang merepresentasikan permukaan objek 3D.

Program juga menampilkan informasi:
- Banyaknya voxel
- Banyaknya vertex dan face hasil voxelization
- Statistik node Octree
- Waktu eksekusi
- Path file output

---

## Requirement Program dan Instalasi

### Perangkat Lunak:
- Compiler C (disarankan: gcc)

### Sistem Operasi:
- Windows (MinGW / MSYS2 / WSL)
- Linux
- macOS

### Instalasi Compiler (Windows - MSYS2):
pacman -S mingw-w64-x86_64-gcc

---

## Cara Mengkompilasi Program

### Struktur Program

├── src/

│   ├── cow.obj

│   ├── line.obj

│   ├── pumpkin.obj

│   ├── teapot.obj

│   ├── tetra.obj

│   ├── Tucil 2.c

│   ├── Tucil.exe

├── test/

│   ├── cow-hasil.obj

│   ├── line-hasil.obj

│   ├── pumpkin-hasil.obj

│   ├── teapot-hasil.obj

│   ├── tetra-hasil.obj

### Kompilasi Program

Masuk ke folder src:
cd src

Kemudian jalankan:
gcc "Tucil 2.c" -o Tucil.exe -O2 -lm

---

## Cara Menjalankan dan Menggunakan Program

### Menjalankan Program

Dari folder src:
.\Tucil.exe <nama_file.obj> <max_depth>

Contoh:
.\Tucil.exe pumpkin.obj 6

---

### Penjelasan Input

- <nama_file.obj>  
  File objek 3D yang akan divoxelize (harus berada di folder src)

- <max_depth>  
  Kedalaman maksimum Octree (menentukan resolusi voxel)

---

### Penjelasan Output

Program akan menghasilkan file .obj hasil voxelization di folder:
../test/

Contoh:
test/pumpkin-hasil.obj

---

### Informasi yang Ditampilkan

Program akan menampilkan di terminal:

- Banyaknya voxel yang terbentuk
- Banyaknya vertex dan face
- Statistik node Octree per depth
- Statistik node yang tidak ditelusuri (pruned)
- Kedalaman Octree
- Waktu eksekusi
- Lokasi file output

---

## Author

Nama: Farrell

NIM: 13524046  
