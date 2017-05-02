"""
GQCNN training script using DeepOptimizer
Author: Vishal Satish
"""
from gqcnn import GQCNN, DeepOptimizer, GQCNNAnalyzer
from core import YamlConfig

# train_config = YamlConfig('cfg/tools/train_grasp_quality_cnn_dexnet_large.yaml')
train_config = YamlConfig('cfg/tools/train_grasp_quality_cnn.yaml')
gqcnn_config = train_config['gqcnn_config']
analysis_config = YamlConfig('cfg/tools/analyze_gqcnn_performance.yaml')
model_dir = '/home/autolab/Public/data/dex-net/data/models/grasp_quality/gqcnn_vgg_mini_dexnet_robust_eps_replication_01_23_17'

###Possible Use-Cases###

# Use Case 1-Prediction
# gqcnn = GQCNN.load(model_dir)
# gqcnn.predict(images, poses)
# gqcnn.close_session()

# Use Case 2-Training from Scratch
train_gqcnn = GQCNN(gqcnn_config)
deepOptimizer = DeepOptimizer(train_gqcnn, train_config)
with train_gqcnn.get_tf_graph().as_default():
    deepOptimizer.optimize()

# Use Case 3-Fine Tuning
# train_gqcnn = GQCNN.load(model_dir)
# deepOptimizer = DeepOptimizer(train_gqcnn, train_config)
# with train_gqcnn._graph.as_default():
    # deepOptimizer.optimize()

# Analysis
# my_analyzer = GQCNNAnalyzer(analysis_config)
# my_analyzer.analyze()