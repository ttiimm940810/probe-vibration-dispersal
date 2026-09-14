% arrange_blocks.m
% 自動整理 dampedsystem.slx 中所有方塊的位置

modelName = 'dampedsystem';

% 開啟模型（若尚未開啟）
if ~bdIsLoaded(modelName)
    open_system(modelName);
end

% 使用 Simulink 內建自動排列功能
Simulink.BlockDiagram.arrangeSystem(modelName);

% 儲存模型
save_system(modelName);

disp('✅ 方塊整理完成，模型已儲存！');
