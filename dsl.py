class Prog:
    def __init__(self, expressions):
        self.expressions = expressions

    def pretty_str(self):
        res = ''
        for exp in self.expressions:
            res += f"{exp.pretty_str()}\n"

        return res

    def execute(self):
        results = []

        for exp in expressions:
            results.append(exp.execute())

        return results 


class Map:
    def __init__(self, obj_tp, map_feature, query):
        self.obj_tp = obj_tp
        self.map_feature = map_feature
        self.query = query

        self.result = {}

    def pretty_str(self):
        return f"map({self.obj_tp}, {self.map_feature}, {self.query.pretty_str()})"

    def execute(self):
        # Query must have been executed in real world to get symbolic results
        assert self.query.result != None

        if self.result == {}:
            for obj_inst in self.query.result[self.obj_tp]:
                self.result[obj_inst] = obj_inst[self.map_feature]

        return self.result

class Primitives:
    def __init__(self, prim_tp, prim, prim2=None, prim_op=None):
        self.prim_tp = prim_tp
        self.prim = prim
        self.prim2 = prim2
        self.prim_op = prim_op

        # Result to be filled
        self.result = None

    def pretty_str(self):
        if self.prim_tp == 'real':
            return f"{self.prim}"
        elif self.prim_tp == "op":
            op = ""
            if self.prim_op == "plus":
                op = "+"
            elif self.prim_op == "minus":
                op = "-"
            elif self.prim_op == "mul":
                op = "*"
            elif self.prim_op == "div":
                op = "/"

            return f"{self.prim.pretty_str()} {op} {self.prim2.pretty_str()}"

        else:
            return f"{self.prim.pretty_str()}"

    def execute(self):
        if self.result == None:
            if self.prim_tp == "real":
                self.result = self.prim
            elif self.prim_tp == "op":
                # Execute left and right primitives
                left = 0
                right = 0
                if type(self.prim) in [GetNth, Count, Aggregator]:
                    if self.prim.result == None:
                        self.prim.execute()

                    left = self.prim.result
                else:
                    left = self.prim
                
                if type(self.prim2) in [GetNth, Count, Aggregator]:
                    if self.prim2.result == None:
                        self.prim2.execute()

                    right = self.prim2.result
                else:
                    right = self.prim2

            
                # Perform the operation
                if self.prim_op == "plus":
                    self.result = left + right
                elif self.prim_op == "minus":
                    self.result = left - right
                elif self.prim_op == "mul":
                    self.result = left * right
                elif self.prim_op == "div":
                    self.result = left / right

            else:
                if type(self.prim) in [GetNth, Count, Aggregator]:
                    if self.prim.result == None:
                        self.prim.execute()

                    self.result = self.prim.result

                else:
                    self.result = self.prim

        return self.result


class GetNth:
    def __init__(self, symbolic_list, index):
        self.list = symbolic_list
        self.index = index 

        # Result
        self.result = None

    def pretty_str(self):
        return f"getNth({self.list.pretty_str()}, {self.index})"

    def execute(self):
        # List must have been evaluated
        assert self.list != {}

        if self.result == None:
            key = list(self.list.keys())[self.index]
            self.result self.list[key]

        return self.result


class Count:
    def __init__(self, query, obj_tp):
        self.query = query
        self.obj_tp = obj_tp

        # Result
        self.result = None

    def pretty_str(self):
        return f"count({self.query.pretty_str()}, {self.obj_tp})"

    def execute(self):
        # Query must have been executed in the real world
        assert self.query.result != None

        if self.result == None:
            self.result = len(self.query.result[self.obj_tp])

        return self.result


class Aggregator:
    def __init__(self, agg_tp, symbolic_list):
        self.agg_tp = agg_tp
        self.list = symbolic_list

        # Result
        self.result = None

    def pretty_str(self):
        return f"{self.agg_tp}({self.list.pretty_str})"

    def execute(self):
        # List must have been completed
        assert self.list != {}

        if self.result == None:
            if self.agg_tp == "sum":
                sum = 0
                
                for key in self.list:
                    sum += self.list[key]

                self.result = sum

            elif self.agg_tp == "avg":
                avg = 0

                for key in self.list:
                    avg += self.list[key]

                self.result = avg/len(self.list)

            elif self.agg_tp == "min":
                min = -1

                for key in self.list:
                    if self.list[key] < min or min == -1:
                        min = self.list[key]

                self.result = min

            elif self.agg_tp == "max":
                max = -1

                for key in self.list:
                    if self.list[key] > max or max == -1:
                        max = self.list[key]

                self.result = max

        return self.result


class Query:
    def __init__(self, obj_tp, where_clause, limit=-1):
        self.obj_tp = obj_tp
        self.where_clause = where_clause
        self.limit = limit

        # Result of executing Query
        self.result = None

    def pretty_str(self):
        res = f'find ({self.obj_tp}) where ({where_clause.pretty_str()})'

        if self.limit > 0:
            res += f" [limit {self.limit}]"

        return res

    def set_result(self, result):
        self.result = result


class WhereClause:
    def __init__(self, where_tp, obj_tp, sub_where_clause=None, obj_tp2=None, 
        scalar_feature=None, scalar_param=None, scalar_comparator=None, enum_feature=None, enum_param=None
        spatial_relation=None)

        self.where_tp = where_tp
        self.obj_tp = obj_tp
        self.sub_where_clause = sub_where_clause
        self.obj_tp2 = obj_tp2
        self.scalar_feature = scalar_feature
        self.scalar_param = scalar_param
        self.scalar_comparator = scalar_comparator
        self.enum_feature = enum_feature
        self.enum_param = enum_param
        self.spatial_relation = spatial_relation

    def pretty_str(self):
        if self.where_tp == "feature_enum":
            return f"{self.enum_feature}({self.obj_tp}) = {self.enum_param}"

        elif self.where_tp == "feature_scalar":
            scalar_comp = ""
            if self.scalar_comparator == "Lt":
                scalar_comp = "<"
            elif self.scalar_comparator == "Leq":
                scalar_comp = "<="
            elif self.scalar_comparator == "Eq":
                scalar_comp = "="
            elif self.scalar_comparator == "Geq":
                scalar_comp = ">="
            elif self.scalar_comparator == "Gt":
                scalar_comp = ">"

            return f"{self.scalar_feature}({self.obj_tp}) {scalar_comp} {self.scalar_param}"

        elif self.where_tp == "max":
            return f"max({self.feature_scalar}({self.obj_tp}))"

        elif self.where_tp == "min":
            return f"min({self.feature_scalar}({self.obj_tp}))"

        elif self.where_tp == "spatial_rel":
            return f"{self.spatial_relation}({self.obj_tp}, {self.obj_tp2})"

        elif self.where_tp == "and":
            return f"{self.sub_where_clause[0].pretty_str()} /\ {self.sub_where_clause[1].pretty_str()}"

        elif self.where_tp == "or":
            return f"{self.sub_where_clause[0].pretty_str()} \/ {self.sub_where_clause[1].pretty_str()}"

        elif self.where_tp == "not":
            return f"!({self.sub_where_clause[0].pretty_str()})"

        elif self.where_tp == "true":
            return "true"