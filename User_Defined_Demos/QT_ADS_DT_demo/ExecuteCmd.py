import io
import sys
from contextlib import redirect_stdout, redirect_stderr
import inspect

class ExecuteCmd:
    def __init__(self,main_window=None, direct_env_dialog=None, adsCommunicatorDialog=None, process_controller=None):
        self.main_window = main_window
        # self.direct_env_dialog = direct_env_dialog
        # 安全环境设置，可以根据需求修改
        self.local_context = {
            "__builtins__": {"print": print, "range": range, "str": str, "bool": bool},  # 允许使用print和range等函数
            "main_window": main_window,
            # "direct_env_dialog": direct_env_dialog,
            # "adsCommunicatorDialog": adsCommunicatorDialog,
            # "process_controller": process_controller,
        }

    def execute_command(self, command):
        """执行命令并返回输出结果"""
        output = io.StringIO()  # 创建一个字符串流来捕获输出
        error = False
        if ' -h' in command:
            base_command = command.split(' -h')[0].strip()
            help_info = self.main_window.completer.complete(base_command)
            return str(help_info), False
        try:
            with redirect_stdout(output), redirect_stderr(output):
                # 安全执行命令，这里可以根据实际需要添加更多的全局或局部变量
                exec(command, self.local_context, {})
        except Exception as e:
            # 捕获并记录执行中的异常
            output.write(f"Error: {str(e)}\n")
            error = True

        result = output.getvalue()
        output.close()
        return result, error

    def add_function(self, name, func):
        self.local_context[name] = func


# class Completer:
#     def __init__(self, context):
#         self.context = context
#
#     def complete(self, text):
#         suggestions = []
#         if '.' in text:
#             parts = text.split('.')
#             obj = self.context.get(parts[0])
#             if obj and len(parts) > 1:
#                 suggestions.extend(self._suggest_attributes(obj, parts[-1]))
#         else:
#             # 这里的处理保持输入的大小写不变
#             suggestions.extend([key for key in self.context.keys() if key.lower().startswith(text.lower())])
#         return suggestions
#
#
#     def _suggest_attributes(self, obj, prefix, help_mode=False, global_search=False):
#         if global_search:
#             results = []
#             for name, obj in self.context.items():
#                 results.extend(self._suggest_attributes(obj, prefix, help_mode=True))
#             return results
#
#         attributes = []
#         for attr in dir(obj):
#             if prefix.lower() in attr.lower():  # 包含检查，不区分大小写
#                 if help_mode:
#                     value = getattr(obj, attr)
#                     if callable(value) or not inspect.isroutine(value):
#                         attributes.append(f"{attr}: {self._format_help(value)}")
#                 else:
#                     attributes.append(attr)
#         return attributes
#
#     def suggest_attributes_whole(self, obj, prefix):
#         return [attr for attr in dir(obj) if attr.startswith(prefix) and
#                 (callable(getattr(obj, attr)) or not inspect.isroutine(getattr(obj, attr)))]
#
#     def help(self, prefix):
#         suggestions = []
#         for name, obj in self.context.items():
#             if callable(obj) or not inspect.isroutine(obj):
#                 if prefix.lower() in name.lower():
#                     doc = inspect.getdoc(obj) or "No documentation available."
#                     suggestions.append(f"{name}: {doc}")
#         return "\n".join(suggestions) if suggestions else "No matching attributes or methods found."
#     def _format_help(self, value):
#         if callable(value):
#             doc = inspect.getdoc(value)
#             return doc if doc else "No documentation available"
#         return "Attribute value"


class Completer:
    def __init__(self, context):
        self.context = context

    def complete(self, text):
        suggestions = []
        if '.' in text:
            parts = text.split('.')
            obj_name = parts[0]
            obj = self.context.get(obj_name)
            partial_attr = '.'.join(parts[1:])  # 获取可能的属性前缀，如果没有则为空字符串
            if obj:
                if partial_attr:
                    # 进行属性建议
                    suggestions = self._suggest_attributes(obj, partial_attr)
                else:
                    # 如果没有具体属性，列出所有可能的属性和方法
                    suggestions = self._suggest_all_attributes(obj)
        else:
            # 如果没有点号，建议顶级对象
            suggestions = [key for key in self.context.keys() if key.lower().startswith(text.lower())]
        return suggestions

    # def _suggest_attributes(self, obj, prefix):
    #     attributes = []
    #     for attr in dir(obj):
    #         # 将属性名和前缀都转为小写进行比较
    #         if attr.lower().startswith(prefix.lower()):
    #             attributes.append(attr)
    #     return attributes

    def _suggest_attributes(self, obj, parts):
        """ Recursively resolve attributes to support multi-level access. """
        if isinstance(parts, str):
            parts = parts.split('.')

        attribute = parts[0]
        attr_obj = getattr(obj, attribute, None)
        if len(parts) > 1 and attr_obj:
            return self._suggest_attributes(attr_obj, parts[1:])
        elif attr_obj:
            return [attribute + '.' + attr for attr in dir(attr_obj) if not attr.startswith('__')]
        else:
            # Suggest attributes or methods for the current level
            return [attr for attr in dir(obj) if
                    attr.lower().startswith(attribute.lower()) and not attr.startswith('__')]

    def _suggest_all_attributes(self, obj):
        attributes = []
        for attr in dir(obj):
            # 列出所有属性和方法，不区分大小写
            if callable(getattr(obj, attr)) or not inspect.isroutine(getattr(obj, attr)):
                attributes.append(attr)
        return attributes



if __name__ == '__main__':
    cmd = ExecuteCmd()
    result = cmd.execute_command('fun1(5)')
    print(result)
    pass