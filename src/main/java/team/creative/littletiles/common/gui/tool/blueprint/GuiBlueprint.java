package team.creative.littletiles.common.gui.tool.blueprint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map.Entry;
import java.util.function.Consumer;

import net.minecraft.client.gui.GuiGraphics;
import net.minecraft.core.component.PatchedDataComponentMap;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.nbt.EndTag;
import net.minecraft.network.chat.Component;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.api.distmarker.OnlyIn;
import team.creative.creativecore.common.gui.Align;
import team.creative.creativecore.common.gui.GuiControl;
import team.creative.creativecore.common.gui.GuiParent;
import team.creative.creativecore.common.gui.VAlign;
import team.creative.creativecore.common.gui.control.collection.GuiComboBoxFlexible;
import team.creative.creativecore.common.gui.control.parent.GuiLeftRightBox;
import team.creative.creativecore.common.gui.control.simple.GuiButton;
import team.creative.creativecore.common.gui.control.simple.GuiButtonIcon;
import team.creative.creativecore.common.gui.control.simple.GuiLabel;
import team.creative.creativecore.common.gui.control.simple.GuiTextfield;
import team.creative.creativecore.common.gui.control.tree.GuiTree;
import team.creative.creativecore.common.gui.control.tree.GuiTree.GuiTreeSelectionChanged;
import team.creative.creativecore.common.gui.control.tree.GuiTreeItem;
import team.creative.creativecore.common.gui.dialog.DialogGuiLayer.DialogButton;
import team.creative.creativecore.common.gui.dialog.GuiDialogHandler;
import team.creative.creativecore.common.gui.flow.GuiFlow;
import team.creative.creativecore.common.gui.flow.GuiSizeRule.GuiSizeRatioRules;
import team.creative.creativecore.common.gui.flow.GuiSizeRule.GuiSizeRules;
import team.creative.creativecore.common.gui.style.Icon;
import team.creative.creativecore.common.gui.sync.GuiSyncLocal;
import team.creative.creativecore.common.gui.sync.GuiSyncLocalLayer;
import team.creative.creativecore.common.util.inventory.ContainerSlotView;
import team.creative.creativecore.common.util.math.geo.Rect;
import team.creative.creativecore.common.util.text.TextBuilder;
import team.creative.creativecore.common.util.text.TextMapBuilder;
import team.creative.creativecore.common.util.type.itr.FunctionIterator;
import team.creative.littletiles.LittleTilesGuiRegistry;
import team.creative.littletiles.LittleTilesRegistry;
import team.creative.littletiles.api.common.tool.ILittleTool;
import team.creative.littletiles.common.block.little.tile.group.LittleGroup;
import team.creative.littletiles.common.block.little.tile.group.LittleVoxelUtils;
import team.creative.littletiles.common.grid.LittleGrid;
import team.creative.littletiles.common.gui.control.animation.GuiAnimationPanel;
import team.creative.littletiles.common.gui.tool.GuiConfigure;
import team.creative.littletiles.common.gui.tool.blueprint.test.BlueprintTest;
import team.creative.littletiles.common.gui.tool.blueprint.test.BlueprintTestError;
import team.creative.littletiles.common.gui.tool.blueprint.test.BlueprintTestResults;
import team.creative.littletiles.common.gui.tool.blueprint.test.GuiBlueprintTest;
import team.creative.littletiles.common.item.ItemLittleBlueprint;
import team.creative.littletiles.common.structure.registry.gui.LittleStructureGui;
import team.creative.littletiles.common.structure.registry.gui.LittleStructureGuiControl;
import team.creative.littletiles.common.structure.registry.gui.LittleStructureGuiRegistry;

public class GuiBlueprint extends GuiConfigure {
    
    public final GuiSyncLocal<EndTag> CLEAR_CONTENT = getSyncHolder().register("clear_content", tag -> {
        CompoundTag content = new CompoundTag();
        LittleGrid.MIN.set(content);
        ILittleTool.setData(tool.get(), content);
        tool.changed();
        LittleTilesGuiRegistry.OPEN_CONFIG.open(getPlayer());
    });
    
    public final GuiSyncLocal<EndTag> REMOVE_CONTENT = getSyncHolder().register("remove_content", tag -> {
        var data = ILittleTool.getData(tool.get());
        data.remove(ItemLittleBlueprint.CONTENT_KEY);
        tool.get().remove(LittleTilesRegistry.DATA);
        tool.changed();
        LittleTilesGuiRegistry.OPEN_CONFIG.open(getPlayer());
    });
    
    public final GuiSyncLocal<CompoundTag> SAVE = getSyncHolder().register("save", tag -> {
        ILittleTool.setData(tool.get(), tag);
        tool.changed();
        GuiBlueprint.super.closeThisLayer();
    });
    
    public final GuiSyncLocalLayer<GuiBlueprintTest> OPEN_TEST = getSyncHolder().layer("test", tag -> new GuiBlueprintTest());
    public final GuiSyncLocalLayer<GuiBlueprintAdd> OPEN_ADD = getSyncHolder().layer("add", tag -> new GuiBlueprintAdd());
    public final GuiSyncLocalLayer<GuiBlueprintMove> OPEN_MOVE = getSyncHolder().layer("move", tag -> new GuiBlueprintMove());
    public final GuiSyncLocalLayer<GuiBlueprintMerge> OPEN_MERGE = getSyncHolder().layer("merge", tag -> new GuiBlueprintMerge());
    
    public GuiTree tree;
    public GuiComboBoxFlexible<LittleStructureGui> types;
    public GuiParent config;
    public LittleStructureGuiControl control;
    public GuiLabel testReport;
    public GuiParent sidebarButtons;
    @OnlyIn(Dist.CLIENT)
    public GuiBlueprintAnimationStorage storage;
    public GuiBlueprintAnimationHandler animation = new GuiBlueprintAnimationHandler();
    private boolean selectedBefore = true;
    
    public GuiBlueprint(ContainerSlotView view) {
        super("blueprint", view);
        flow = GuiFlow.STACK_X;
        valign = VAlign.STRETCH;
        setDim(new GuiSizeRules().minWidth(500).minHeight(300));
        registerEventChanged(x -> {
            if (x.control.is("type") && tree.selected() != null)
                ((GuiTreeItemStructure) tree.selected()).load();
            else if (x instanceof GuiTreeSelectionChanged sel)
                if (selectedBefore != (sel.selected != null)) {
                    selectedBefore = sel.selected != null;
                    for (GuiControl control : sidebarButtons)
                        if (!control.is("add"))
                            control.setEnabled(selectedBefore);
                }
        });
    }
    
    @Override
    public void becameTopLayer() {
        super.becameTopLayer();
        if (isClient())
            get("animation", GuiAnimationPanel.class).refresh();
    }
    
    @Override
    public boolean saveConfiguration(PatchedDataComponentMap data) {
        return false;
    }
    
    public void buildStructureTree(GuiTree tree, GuiTreeItem parent, LittleGroup group, int index) {
        if (group.isEmpty()) {
            if (!group.children.hasChildren())
                return;
            for (LittleGroup child : group.children.children()) {
                buildStructureTree(tree, parent, child, index);
                index++;
            }
            return;
        }
        
        LittleGroup copy = new LittleGroup(group.hasStructure() ? group.getStructureTag().copy() : null, Collections.EMPTY_LIST);
        copy.addAll(group.getGrid(), new FunctionIterator<>(group, x -> x.copy()));
        for (Entry<String, LittleGroup> extension : group.children.extensionEntries())
            copy.children.addExtension(extension.getKey(), extension.getValue().copy());
        GuiTreeItemStructure item = new GuiTreeItemStructure(this, tree, copy, index);
        parent.addItem(item);
        
        int i = 0;
        for (LittleGroup child : group.children.children()) {
            buildStructureTree(tree, item, child, i);
            i++;
        }
    }
    
    @Override
    public void closeThisLayer() {
        closeWithDialog();
    }
    
    @Override
    public void closeTopLayer() {
        closeWithDialog();
    }
    
    @Override
    public void closed() {
        if (isClient())
            storage.unload();
    }
    
    private void closeWithDialog() {
        if (runTest().success()) {
            CompoundTag nbt = LittleGroup.save(reconstructBlueprint());
            
            if (ItemLittleBlueprint.getContent(tool.get()).equals(nbt)) { // No need to save anything
                super.closeThisLayer();
                return;
            }
            
            GuiDialogHandler.openDialog(getIntegratedParent(), "cancel", translatable("gui.blueprint.cancel.dialog"), (g, b) -> {
                if (b == DialogButton.CANCEL)
                    return;
                if (b == DialogButton.YES)
                    SAVE.send(LittleGroup.save(reconstructBlueprint()));
                GuiBlueprint.super.closeThisLayer();
            }, DialogButton.CANCEL, DialogButton.NO, DialogButton.YES);
        } else {
            GuiDialogHandler.openDialog(getIntegratedParent(), "cancel", translatable("gui.blueprint.cancel.dialog.failed"), (g, b) -> {
                if (b == DialogButton.CONFIRM)
                    GuiBlueprint.super.closeThisLayer();
            }, DialogButton.ABORT, DialogButton.CONFIRM);
        }
    }
    
    @Override
    public void create() {
        if (!isClient())
            return;
        
        flow = GuiFlow.STACK_Y;
        align = Align.STRETCH;
        
        // Load blueprint content
        LittleGroup group = LittleGroup.load(ItemLittleBlueprint.getContent(tool.get()));
        
        GuiParent top = new GuiParent(GuiFlow.STACK_X);
        add(top.setExpandableY());
        
        tree = new GuiTree("overview", false) {
            
            @Override
            public void updateTree() {
                actionOnAllItems(x -> x.updateTitle());
                super.updateTree();
            }
            
        }.setRootVisibility(false).keepSelected();
        
        if (storage == null)
            storage = new GuiBlueprintAnimationStorage(tree);
        
        buildStructureTree(tree, tree.root(), group, 0);
        tree.root().setTitle(Component.literal("root"));
        tree.updateTree();
        
        GuiParent sidebar = new GuiParent(GuiFlow.STACK_Y).setAlign(Align.STRETCH);
        top.add(sidebar.setDim(new GuiSizeRatioRules().widthRatio(0.2F).maxWidth(100)));
        sidebar.add(tree.setExpandableY());
        
        sidebarButtons = new GuiParent(GuiFlow.FIT_X);
        sidebar.add(sidebarButtons.setAlign(Align.CENTER));
        
        sidebarButtons.add(new GuiButton("add", x -> OPEN_ADD.open(new CompoundTag()).init(this)).setTranslate("gui.plus").setAlign(Align.CENTER).setVAlign(VAlign.CENTER).setDim(
            15, 15).setTooltip(new TextBuilder().translate("gui.blueprint.add").build()));
        sidebarButtons.add(new GuiButtonIcon("duplicate", Icon.DUPLICATE, x -> {
            if (tree.selected() == null)
                return;
            tree.selected().getParentItem().addItem(((GuiTreeItemStructure) tree.selected()).duplicate());
            tree.updateTree();
        }).setDim(15, 15).setTooltip(new TextBuilder().translate("gui.blueprint.duplicate").build()));
        sidebarButtons.add(new GuiButton("del", x -> {
            if (tree.selected() == null)
                return;
            GuiDialogHandler.openDialog(getIntegratedParent(), "delete_item", Component.translatable("gui.blueprint.dialog.delete", ((GuiTreeItemStructure) tree.selected())
                    .getTitle()), (g, b) -> {
                        if (b == DialogButton.YES)
                            removeItem((GuiTreeItemStructure) tree.selected());
                    }, DialogButton.NO, DialogButton.YES);
        }).setTranslate("gui.del").setAlign(Align.CENTER).setVAlign(VAlign.CENTER).setDim(15, 15).setTooltip(new TextBuilder().translate("gui.blueprint.delete").build()));
        
        sidebarButtons.add(new GuiButtonIcon("move", Icon.MOVE, x -> OPEN_MOVE.open(new CompoundTag()).init(this)).setDim(15, 15).setTooltip(new TextBuilder().translate(
            "gui.blueprint.move").build()));
        
        sidebarButtons.add(new GuiButtonIcon("up", Icon.ARROW_UP, x -> tree.moveUp()).setDim(15, 15).setTooltip(new TextBuilder().translate("gui.blueprint.moveup").build()));
        sidebarButtons.add(new GuiButtonIcon("down", Icon.ARROW_DOWN, x -> tree.moveDown()).setDim(15, 15).setTooltip(new TextBuilder().translate("gui.blueprint.movedown")
                .build()));
        sidebarButtons.add(new GuiButtonIcon("merge", Icon.MERGE, x -> OPEN_MERGE.open(new CompoundTag()).init(this)).setDim(15, 15).setTooltip(new TextBuilder().translate(
            "gui.blueprint.merge").build()));
        
        GuiParent topCenter = new GuiParent(GuiFlow.STACK_Y).setAlign(Align.STRETCH);
        top.add(topCenter.setDim(new GuiSizeRatioRules().widthRatio(0.4F).maxWidth(400)).setExpandableY());
        
        // Actual blueprint configuration
        types = new GuiComboBoxFlexible<>("type", new TextMapBuilder<LittleStructureGui>().addComponent(LittleStructureGuiRegistry.registered(), x -> x.translatable()), x -> x
                .translatable());
        topCenter.add(types);
        config = new GuiParent("config", GuiFlow.STACK_Y).setAlign(Align.STRETCH);
        topCenter.add(config.setExpandableY());
        config.registerEventChanged(x -> {
            if (x.control.is("name") && tree.selected() instanceof GuiTreeItemStructure item)
                item.onNameChanged((GuiTextfield) x.control);
        });
        
        top.add(new GuiAnimationPanel(tree, storage, true, animation));
        
        GuiLeftRightBox bottom = new GuiLeftRightBox();
        add(bottom.setVAlign(VAlign.CENTER).setExpandableX());



        //test
        // ----- 旋转控制面板 -----
        GuiParent rotateContainer = new GuiParent(GuiFlow.FIT_X);
        rotateContainer.setVAlign(VAlign.CENTER);
        rotateContainer.add(new GuiLabel("rotLabel").setTranslate("gui.blueprint.rotate.label"));

// 三个角度输入框（单位：度）
        GuiTextfield yawField = new GuiTextfield("yaw", "0").setDim(40, 15);
        yawField.setTooltip("yaw");
        rotateContainer.add(yawField);

        GuiTextfield pitchField = new GuiTextfield("pitch", "0").setDim(40, 15);
        pitchField.setTooltip("pitch");
        rotateContainer.add(pitchField);

        GuiTextfield rollField = new GuiTextfield("roll", "0").setDim(40, 15);
        rollField.setTooltip("roll");
        rotateContainer.add(rollField);

// 应用旋转按钮
        GuiButton applyRotate = (GuiButton) new GuiButton("applyRotate", x -> {
            try {
                float yawDeg = Float.parseFloat(yawField.getText());
                float pitchDeg = Float.parseFloat(pitchField.getText());
                float rollDeg = Float.parseFloat(rollField.getText());

                // 转换为弧度
                float yaw = (float) Math.toRadians(yawDeg);
                float pitch = (float) Math.toRadians(pitchDeg);
                float roll = (float) Math.toRadians(rollDeg);

                // 加载当前蓝图内容
                LittleGroup groupTemp = LittleGroup.load(ItemLittleBlueprint.getContent(tool.get()));
                if (groupTemp.isEmptyIncludeChildren()) {
                    // 提示无内容
                    return;
                }

                // 执行体素旋转（注意：该方法内部会体素化并任意角度旋转）
                LittleGroup rotated = LittleVoxelUtils.rotateVoxels(groupTemp, yaw, pitch, roll);

                // 保存到物品
                CompoundTag stackTag = ILittleTool.getData(tool.get());
                stackTag.put(ItemLittleBlueprint.CONTENT_KEY, LittleGroup.save(rotated));
                ILittleTool.setData(tool.get(), stackTag);
                tool.changed();

                // 刷新界面
                refresh();

            } catch (NumberFormatException e) {
                // 输入无效，弹出提示（可通过 GuiDialogHandler 显示错误）
                GuiDialogHandler.openDialog(getIntegratedParent(), "rotate_error",
                        Component.translatable("gui.blueprint.rotate.error"),
                        (g, b) -> {}, DialogButton.CONFIRM);
            }
        }).setTranslate("rotate").setAlign(Align.CENTER).setVAlign(VAlign.CENTER);

        rotateContainer.add(applyRotate);
        bottom.addRight(rotateContainer);
        //



        bottom.addLeft(new GuiButton("cancel", x -> closeThisLayer()).setTranslate("gui.cancel"));
        bottom.addLeft(new GuiButton("selection", x -> {
            GuiDialogHandler.openDialog(getIntegratedParent(), "remove_content", Component.translatable("gui.blueprint.dialog.clear"), (g, b) -> {
                if (b == DialogButton.YES)
                    REMOVE_CONTENT.send(EndTag.INSTANCE);
            }, DialogButton.NO, DialogButton.YES);
        }).setTranslate("gui.blueprint.selection"));
        bottom.addLeft(new GuiButton("clear", x -> {
            GuiDialogHandler.openDialog(getIntegratedParent(), "clear_content", Component.translatable("gui.blueprint.dialog.clear"), (g, b) -> {
                if (b == DialogButton.YES)
                    CLEAR_CONTENT.send(EndTag.INSTANCE);
            }, DialogButton.NO, DialogButton.YES);
        }).setTranslate("gui.blueprint.clear"));
        
        bottom.addRight(testReport = new GuiLabel("report").setTitle(Component.empty()));
        bottom.addRight(new GuiButton("check", x -> OPEN_TEST.open(new CompoundTag()).init(this)).setTranslate("gui.blueprint.test"));
        bottom.addRight(new GuiButton("save", x -> {
            if (runTest().success())
                SAVE.send(LittleGroup.save(reconstructBlueprint()));
        }).setTranslate("gui.save"));
        
        tree.selectFirst();
    }
    
    @Override
    public void tick() {
        if (!isClient())
            return;
        
        super.tick();
        animation.tick();
        if (storage != null)
            storage.tick();
    }

    public void refresh() {
        // 清空树根并重建
        tree.root().clear();
        LittleGroup group = LittleGroup.load(ItemLittleBlueprint.getContent(tool.get()));
        buildStructureTree(tree, tree.root(), group, 0);
        tree.updateTree();
        tree.selectFirst();

        // 重置测试报告
        testReport.setTitle(Component.empty());
    }
    
    @Override
    public void render(GuiGraphics graphics, Rect controlRect, Rect realRect, double scale, int mouseX, int mouseY) {
        storage.renderTick();
        super.render(graphics, controlRect, realRect, scale, mouseX, mouseY);
    }
    
    public void removeItem(GuiTreeItemStructure item) {
        if (item == null)
            return;
        item.getParentItem().removeItem(item);
        tree.updateTree();
        tree.selectFirst();
    }
    
    public void actionOnAllItems(Consumer<GuiTreeItemStructure> con) {
        for (GuiTreeItem item : tree.allItems())
            if (item instanceof GuiTreeItemStructure s)
                con.accept(s);
    }
    
    public BlueprintTestResults runTest() {
        if (tree.selected() != null)
            ((GuiTreeItemStructure) tree.selected()).save();
        BlueprintTestResults results = BlueprintTest.STANDARD.test(this);
        actionOnAllItems(x -> x.clearErrors());
        
        if (results.success()) {
            testReport.setTitle(translatable("gui.blueprint.test.result.success"));
            get("check", GuiButton.class).setTranslate("gui.blueprint.test");
        } else {
            for (BlueprintTestError error : results)
                for (GuiTreeItemStructure item : error)
                    item.addError(error);
                
            String title = translate("gui.blueprint.test.result.fail") + " ";
            if (results.errorCount() == 1)
                title += translate("gui.blueprint.test.error.single");
            else
                title += translate("gui.blueprint.test.error.multiple", results.errorCount());
            testReport.setTitle(Component.literal(title));
            get("check", GuiButton.class).setTranslate("gui.blueprint.solve");
        }
        
        actionOnAllItems(x -> {
            x.updateTitle();
            x.updateTooltip();
        });
        
        reflow();
        
        return results;
    }
    
    protected LittleGroup reconstructBlueprint(GuiTreeItemStructure item) {
        Consumer<GuiTreeItemStructure> finalizer = LittleStructureGuiRegistry.getFinalizer(item.getStructureType());
        if (finalizer != null)
            finalizer.accept(item);
        
        List<LittleGroup> children = new ArrayList<>();
        for (GuiTreeItem child : item.items())
            children.add(reconstructBlueprint((GuiTreeItemStructure) child));
        CompoundTag nbt;
        if (item.structure == null)
            nbt = null;
        else {
            nbt = new CompoundTag();
            item.structure.save(nbt, provider());
        }
        return new LittleGroup(nbt, item.group.copyExceptChildren(), children);
    }
    
    protected LittleGroup reconstructBlueprint() {
        if (tree.root().itemsCount() == 1)
            return reconstructBlueprint((GuiTreeItemStructure) tree.root().items().iterator().next());
        List<LittleGroup> children = new ArrayList<>();
        for (GuiTreeItem child : tree.root().items())
            children.add(reconstructBlueprint((GuiTreeItemStructure) child));
        return new LittleGroup((CompoundTag) null, children);
    }
    
}
