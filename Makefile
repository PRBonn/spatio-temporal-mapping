.PHONY: build clean run test python



# --- VARIABLES ---
ref_number = 1
query_number = 2
row_number = 3
dataset_main_folder = "dataset/"
visualize = 0


# --- MAIN COMMANDS ---
build:
	@cmake -Bbuild cpp/glasshouse_mapping/
	@cmake --build build -j$(nproc)

clean:
	rm -rf build 

clean_deps:
	rm -rf cpp/glasshouse_mapping/libtorch 


# --- TARGETS ---
mapping: 
	./build/apps/mapping -d $(dataset_main_folder) --dataset-number $(ref_number) --row-number $(row_number) --use-wheel-odometry 1 --visualize $(visualize)

mapping_aligned: 
	./build/apps/slam -d $(dataset_main_folder) --ref-number $(ref_number) --query-number $(query_number) --row-number $(row_number) --use-wheel-odometry 1 --visualize $(visualize)

associate: 
	./build/apps/associate -d $(dataset_main_folder) --ref-number $(ref_number) --query-number $(query_number) --row-number $(row_number) --visualize $(visualize)

sp_deform:
	./build/apps/sp_deform -d $(dataset_main_folder) --ref-number $(ref_number) --query-number $(query_number) --row-number $(row_number) --visualize $(visualize)

my_deform: sp_deform
	./build/apps/cilantro_non_rigid_icp -d $(dataset_main_folder) --ref-number $(ref_number) --query-number $(query_number) --row-number $(row_number) --after-sp 1 --visualize $(visualize)

cilantro_deform: 
		./build/apps/cilantro_non_rigid_icp -d $(dataset_main_folder) --ref-number $(ref_number) --query-number $(query_number) --row-number $(row_number) --after-sp 0 --visualize $(visualize)

evaluate_superpoint: 
	@./build/apps/evaluate -d $(dataset_main_folder) --ref-number $(ref_number) --query-number $(query_number) --row-number $(row_number) --method_name 'superpoint' --visualize $(visualize)

evaluate_myapproach: 
	@./build/apps/evaluate -d $(dataset_main_folder) --ref-number $(ref_number) --query-number $(query_number) --row-number $(row_number) --method_name 'myapproach' --visualize $(visualize)

evaluate_cilantro: 
	@./build/apps/evaluate -d $(dataset_main_folder) --ref-number $(ref_number) --query-number $(query_number) --row-number $(row_number) --method_name 'cilantro' --visualize $(visualize)

